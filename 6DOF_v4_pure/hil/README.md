# HIL — Hardware-In-the-Loop (M130)

مجلد اختبار HIL **مستقل تماماً** عن PIL. يُشغّل محاكاة 6DOF على اللابتوب،
ويُرسل بيانات حسّاسات وهمية للهاتف عبر TCP، بينما **السيرفوهات حقيقية**
متصلة عبر CAN (Waveshare USB_CAN_A → 4 سيرفوهات XQPOWER).

## الفرق الجوهري عن PIL

في PIL: الأمر من MPC يذهب للمحاكاة مباشرة (δ مثالي).

في HIL: **حلقة مغلقة ميكانيكياً**:

```
MPC (Android/PX4)
  ↓ actuator_servos
xqpower_can driver
  ↓ CAN frames (XQPOWER protocol)
USB_CAN_A (CH340, VID=0x1A86)
  ↓
4 سيرفوهات حقيقية
  ↓ قراءة الزاوية الفعلية (encoder)
CAN feedback frames
  ↓
xqpower_can → debug_array (SRV_FB, id=1)
  ↓ DEBUG_FLOAT_ARRAY stream @ 20Hz
MAVLink (TCP 5760)
  ↓
mavlink_bridge_hil ← يقرأ fb_deg[0..3]
  ↓
Simulation يستخدم الزاوية المقاسة بدل الأمر
```

**النتيجة**: ديناميكا السيرفو الحقيقية (slew rate, backlash, CAN latency)
تؤثر فعلياً على مسار المحاكاة.

## المتطلبات

- الهاتف: Samsung S23 Ultra مع `com.ardophone.px4v17`
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

- `results/hil_flight.csv` — مسار الرحلة (الحالة الديناميكية)
- `results/hil_flight_servo.csv` — **جوهر HIL**: سجلّ cmd/fb/err لكل سيرفو
- `results/hil_timing.csv` — توقيت MHE/MPC/Cycle
- `results/baseline_flight.csv` — المرجع (Python MPC على اللابتوب)

## التحقق من عمل الفيدباك

بعد تشغيل HIL، يجب أن ترى:

```
[HIL] Servo feedback: N frames, fallback_to_cmd=K steps, online_mask=0x0F, tx_fail=0
```

- `N > 0` — الفيدباك يصل
- `online_mask=0x0F` — جميع السيرفوهات الأربعة متصلة
- `fallback_to_cmd` قليل — المحاكاة استخدمت الفيدباك بأغلب الخطوات

إذا `N=0`: تحقق من:
1. `adb forward tcp:5760` مُفعّل
2. `xqpower_can` يعمل: `adb logcat | grep -i xqpower`
3. CAN متصل: `online_mask != 0`

## الإعدادات المهمة في `hil_config.yaml`

```yaml
hil:
  use_servo_feedback: true              # استخدم fb_deg بدل الأمر
  servo_feedback_timeout_ms: 200.0      # timeout قبل fallback للأمر
  require_all_servos_online: false      # إن true: يوقف إذا أحد السيرفوهات offline
```

## السلامة

قبل أول تشغيل كامل:
1. اختبر السيرفوهات على bench بدون حمل ميكانيكي
2. تحقق من اتجاه الدوران عبر `XQCAN_REV` parameter
3. تحقق من حدود الزوايا عبر `XQCAN_LIMIT`
4. أبقِ مصدر الطاقة قابلاً للفصل السريع (E-stop)

## الملفات

- `mavlink_bridge_hil.py` — جسر MAVLink (يستقبل SRV_FB)
- `hil_runner.py` — المشغّل الرئيسي (baseline + HIL + compare)
- `hil_config.yaml` — الإعدادات
- `results/` — المخرجات

