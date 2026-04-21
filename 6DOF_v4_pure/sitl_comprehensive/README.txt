Comprehensive SITL Test Suite — اختبار SITL الموسّع
=====================================================

مجلد مستقل تماماً. لا يعدّل أي شيء في `sitl/` الأصلي.

Entry point:
    python run_comprehensive_test.py --help

Modules:
    config_matrix.py       — يولّد مصفوفة السيناريوهات (bearing × pitch × distance × ...)
    scenario_runner.py     — يشغّل سيناريو واحد (baseline + optional SITL) مع override للـ YAML
    failure_injector.py    — يحقن أعطال (NaN, GPS loss, attitude freeze, EKF dropout)
    parity_checker.py      — يقارن MHE vs ground-truth و Python vs C++ (عند توفّر SITL CSV)
    multi_arm_test.py      — دورات arm→fly→disarm المتتالية
    control_authority.py   — يفحص استهلاك max deflection
    chi_wrap_test.py       — هدف خلف الصاروخ لكشف angle-wrap
    enhanced_metrics.py    — معايير نجاح موسّعة (بدل الـ 4 الضيّقة)
    coverage_report.py     — تقرير HTML شامل يحسب coverage الأخطاء المعروفة

Outputs:
    results/<timestamp>/
        matrix_summary.csv
        per_scenario/*.csv
        coverage_report.html
        failures.log
