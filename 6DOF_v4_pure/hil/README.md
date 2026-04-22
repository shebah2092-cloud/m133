# HIL — Hardware-In-the-Loop (M130)

> **حالة التنفيذ:** هذا الإصدار يدعم الآن وضعين عبر `hil.mode`:
>
> - **`monitor_only` (default)** — ديناميكا المحاكاة تستخدم أمر MPC الخام
>   ممرّراً عبر نموذج سيرفو Python من الدرجة الأولى (τ=15ms). فيدباك
>   CAN للتسجيل وفحص السلامة فقط. هذا السلوك التاريخي ويُطابق الطيران
>   الحقيقي حيث EKF يستنتج الحالة من IMU فقط ولا يرى زاوية السيرفو.
> - **`closed_loop`** — الزاوية المقاسة من السيرفوهات الحقيقية (SRV_FB)
>   تُحقن مباشرة في الأيروديناميكا، ويُعطَّل نموذج Python لتفادي مضاعفة
>   التأخير. هذا يكشف سلوك العتاد الفعلي (backlash، slew، CAN latency،
>   jitter) على المسار.
>
> لتفعيل closed_loop أضف `mode: closed_loop` في `hil_config.yaml`. لا
> تغيير في أي سلوك قائم إذا لم تُضف `mode` صراحةً.

مجلد اختبار HIL مستقل عن PIL: يُشغّل محاكاة 6DOF على اللابتوب، ويُرسل
حسّاسات اصطناعية للهاتف عبر TCP 4560، بينما MPC يعمل على هاتف Android
بـ PX4 الحقيقي (C++)، والسيرفوهات الحقيقية متصلة عبر CAN
(Waveshare USB_CAN_A → 4 سيرفوهات XQPOWER).

## ما يقيسه هذا الاختبار فعلياً

1. **صحّة MPC الـ C++ على PX4** في دورة طيران كاملة مقابل baseline Python.
2. **هل العتاد يتّبع الأوامر؟** — فيدباك CAN يُقارن مع الأمر كفحص سلامة
   (tracking error، online_mask، tx_fail، latency).
3. **سلامة stack الاتصالات** — MAVLink, TCP, debug_array, CAN transport.

## ما لا يقيسه هذا الاختبار (في الوضع الحالي)

- **أثر ديناميكا السيرفو الفعلية على المسار** — المسار محسوب من نموذج
  Python، لا من العتاد. لذا backlash، slew، CAN latency، torque ceiling
  غير ظاهرة في الأيروديناميكا.
- عتبة `servo_tracking` في `hil_config.yaml` تُقارن مخرج **نموذج Python**
  بالأمر (ليس العتاد). لا تفشل عملياً ولا تكشف أخطاء hardware.

لقياس أداء العتاد فعلياً، راجع أعمدة `fin_can_*` في
`results/hil_flight_servo.csv` (هذه الزاوية الحقيقية المقاسة).

## تدفق البيانات الفعلي (كما هو في الكود)

```
PX4 MPC (Android)
  ↓ uORB: actuator_outputs_sim  (أو actuator_servos)
XqpowerCan driver  [100Hz]
  ↓ CAN TX (XQPOWER protocol, 500 kbps)
USB_CAN_A (CH340, VID=0x1A86)
  ↓
4 سيرفوهات حقيقية
  ↓ PDO auto-report (كل 50ms، ≈20Hz/servo)
USB-Serial RX ← CAN RX
  ↓ XqpowerCan يجمّع ويَنشر
debug_array (name="SRV_FB", id=1)  [≤100Hz]
  ↓ MAVLink DEBUG_FLOAT_ARRAY (TCP 5760)
Python mavlink_bridge_hil.py
  ↓
        ┌─────────────────────────────────┐
        │                                 │
   [مسار الرصد فقط]                   [مسار الديناميكا]
   _servo_fb_rad ↓                    _last_controls[:4] (أمر MPC)
   • تسجيل CSV (fin_can_*)              ↓
   • فحص خطأ تتبع (10° حد السلامة)     ActuatorModel Python (τ=15ms)
   • online_mask، tx_fail               ↓
   • معايرة صفر                        aero forces + 6DOF RK4
   • لا يُحقن في الديناميكا             ↓
                                   HIL_STATE_QUATERNION, HIL_SENSOR
                                        ↓ TCP 4560
                                      PX4 EKF2 → MPC التالي
```

## المتطلبات

- الهاتف: Android (S23 Ultra مُختبر) مع `com.ardophone.px4v17`
- Waveshare USB_CAN_A (CH340) عبر USB-C OTG hub
- 4 سيرفوهات XQPOWER على CAN bus (عناوين 1..4)
- مصدر تغذية للسيرفوهات (12V/24V حسب النوع)
- laptop + الهاتف في نفس USB (ADB)

## الإعداد

```bash
adb reverse tcp:4560 tcp:4560    # simulator → phone
adb forward tcp:5760 tcp:5760    # phone → mavlink readback (SRV_FB + TIMING)
```

## التشغيل

```bash
cd 6DOF_v4_pure/hil
source ../../.venv/bin/activate

# 1) كامل: baseline + HIL + مقارنة
python hil_runner.py

# 2) HIL فقط (ينتظر اتصال PX4)
python mavlink_bridge_hil.py

# 3) baseline فقط
python hil_runner.py --baseline-only
```

ثم على الهاتف: افتح التطبيق → اضغط **Start PX4**.

## المخرجات

- `results/hil_flight.csv` — مسار الرحلة (الحالة الديناميكية، fin angle
  الذي دخل الأيروديناميكا هو مخرج نموذج Python لا العتاد).
- `results/hil_flight_servo.csv` — سجلّ مراقبة العتاد: `cmd/fb/err` لكل
  سيرفو، online_mask، tx_fail. **لقياس أداء العتاد الحقيقي استخدم هذا
  الملف.**
- `results/hil_timing.csv` — توقيت MHE/MPC/Cycle على PX4.
- `results/baseline_flight.csv` — المرجع (Python MPC على اللابتوب، بدون
  عتاد).

## التحقق من عمل مسار الرصد

بعد تشغيل HIL، يجب أن ترى:

```
[HIL] Servo feedback: N frames, fallback_to_cmd=K steps, online_mask=0x0F, tx_fail=0
```

- `N > 0` — فيدباك CAN يصل (مسار الرصد يعمل).
- `fallback_to_cmd=0` في الوضع الحالي دائماً — العدّاد محجوز لـ Phase 2
  (closed-loop) حيث سيعدّ الخطوات التي عادت إلى cmd fallback عند stale
  feedback. في monitor-only لا يوجد fallback لأن الديناميكا لا تستخدم
  الفيدباك أصلاً.
- `online_mask=0x0F` — جميع السيرفوهات الأربعة متصلة.
- `tx_fail=0` — لا فشل إرسال CAN.

إذا `N=0`:
1. `adb forward tcp:5760` مُفعّل؟
2. `xqpower_can` يعمل؟ `adb logcat | grep -i xqpower`
3. CAN متصل فعلياً؟ `online_mask != 0`

> ملاحظة: حتى لو `N=0`، الطيران يكتمل لأن الديناميكا لا تعتمد على الفيدباك
> في الوضع الحالي. ستَرى فقط أن أعمدة `fin_can_*` في CSV فارغة/صفرية.

## الإعدادات المهمة في `hil_config.yaml`

```yaml
hil:
  mode: monitor_only                    # monitor_only | closed_loop
  use_servo_feedback: true              # تمكين مسار الرصد (تسجيل + سلامة)
  servo_feedback_timeout_ms: 200.0      # يُعدّ قياس CAN قديماً بعد 200ms
  servo_feedback_abort_ms: 500.0        # closed_loop: توقّف المحاكاة
  servo_feedback_grace_ms: 500.0        # closed_loop: فترة سماح بدء الطيران
  require_all_servos_online: false      # true: يوقف إذا أي سيرفو offline
  servo_auto_zero: true                 # معايرة صفر تلقائية قبل الطيران
```

**`monitor_only` (default)**: تبديل `use_servo_feedback: false` يُعطّل
قراءة SRV_FB وتسجيلها — لا يغيّر ديناميكا المحاكاة.

**`closed_loop`**: السيرفوهات الحقيقية تقود الأيروديناميكا. سلوك السقوط
عند فقدان الفيدباك:
1. خلال `servo_feedback_grace_ms` الأولى من الطيران: إذا لم يُستلم أي
   فيدباك بعد، نستخدم أمر MPC مؤقتاً (`fin_source=cmd`) — يسمح بالإقلاع
   إذا تأخّر أول PDO.
2. بعد الـ grace: إذا شاخ الفيدباك > `servo_feedback_timeout_ms` لكن
   < `servo_feedback_abort_ms`، نحتفظ بآخر زاوية CAN معروفة
   (`fin_source=hold`).
3. إذا تجاوز الشيخوخة `servo_feedback_abort_ms` → إيقاف محاكاة فوري
   (bench abort، ليس DO_FLIGHTTERMINATION).

عمود `fin_source` في `hil_flight.csv` يوضّح القرار في كل خطوة.

## السلامة

قبل أول تشغيل كامل:
1. اختبر السيرفوهات على bench بدون حمل ميكانيكي.
2. تحقق من اتجاه الدوران عبر `XQCAN_REV` parameter.
3. تحقق من حدود الزوايا عبر `XQCAN_LIMIT`.
4. أبقِ مصدر الطاقة قابلاً للفصل السريع (E-stop).

## المسار المستقبلي (Phase 3+)

Phase 2 (هذا الإصدار) يدعم `closed_loop` بحقن الزاوية المقاسة في
الديناميكا. التحسينات المتبقية:

1. **Phase 3** — رفع معدل PDO auto-report من 50ms (20Hz) إلى 10ms
   (100Hz) في `XqpowerCan.cpp` لتقليل transport delay في closed_loop.
2. **Phase 4** — preflight fin exerciser: بعد الـ auto-zero وقبل الإقلاع،
   تشغيل السيرفوهات عبر مدى ±15° للتحقق من صحة الاستجابة قبل الحلقة
   المغلقة.
3. **Phase 5** — validation report يقارن real-flight vs closed-loop HIL
   vs monitor_only على نفس السيناريو.

خطة التنفيذ الكاملة موثّقة في تقرير إعادة تصميم HIL.

## الملفات

- `mavlink_bridge_hil.py` — جسر MAVLink (يستقبل sensors من sim، يُرسل
  HIL_STATE_QUATERNION، يقرأ HIL_ACTUATOR_CONTROLS، يقرأ SRV_FB للرصد).
- `hil_runner.py` — المشغّل الرئيسي (baseline + HIL + compare).
- `hil_config.yaml` — الإعدادات.
- `results/` — المخرجات.
