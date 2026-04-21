# ملاحظات مقارنة PIL

مستقل تماماً عن اختبار SITL.

## الملفات

```
pil/
├── mavlink_bridge_pil.py   ← جسر MAVLink مستقل (لا يستورد من sitl/)
├── pil_runner.py           ← baseline + PIL + مقارنة
├── pil_config.yaml         ← هدف + عتبات
└── results/
    ├── baseline_flight.csv ← مرجع Python + MPC داخلي
    ├── pil_flight.csv      ← من جهاز ARM64 الفعلي
    └── pil_timing.csv      ← إحصائيات توقيت من الهدف
```

## المقارنة الوحيدة

**baseline ↔ PIL** — تحقق من أن PX4 على ARM64 يطابق المحاكاة المستقلة.

| المقياس | العتبة |
|---|---|
| velocity_mae | < 5.0 m/s |
| attitude_max | < 5.0° |
| altitude_mae | < 100 m |
| CEP | < 50 m |

**timing** — يُقرأ من `pil_timing.csv` إن تمّ نشره من الهدف:
- `mpc_us p95 < 10 ms`
- `cycle_us p95 < 15 ms`
- `deadline_miss = 0`

## تشغيل

```bash
# 1) شغّل PX4 على الجهاز الهدف مع عنوان IP الخاص بالـ PC
# 2) ثم على الـ PC:
cd 6DOF_v4_pure/pil
python pil_runner.py                    # الكامل
python pil_runner.py --baseline-only    # baseline فقط
python pil_runner.py --pil-only         # PIL فقط
python pil_runner.py --compare-only     # مقارنة من CSV موجودة
```

## متطلبات الهدف

لقياس التوقيت، يجب أن ينشر `rocket_mpc` على الجهاز إحدى الطريقتين:

1. **`DEBUG_VECT`** باسم `"TIMING"` حيث `x=mhe_us`, `y=mpc_us`, `z=cycle_us`.
2. أو **`NAMED_VALUE_FLOAT`** بالأسماء: `mhe_us`, `mpc_us`, `cycle_us`.

إن لم يُنشر شيء، يُنشَأ `pil_timing.csv` فارغاً والمقارنة تقتصر على صحة المنطق.

