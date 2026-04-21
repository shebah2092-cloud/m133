#!/usr/bin/env python3
"""
run_comprehensive_test.py
=========================
نقطة الدخول للاختبار الشامل.

الاستخدامات:
    # mini sweep للتطوير (~10 سيناريو، بلا PX4)
    python run_comprehensive_test.py --quick

    # التغطية الكاملة (~45 سيناريو، Python only)
    python run_comprehensive_test.py

    # مع parity subset (يتطلب PX4 binary)
    python run_comprehensive_test.py --with-sitl --px4-bin /path/to/px4

    # تشغيل سيناريو واحد بالاسم
    python run_comprehensive_test.py --only sweep_bearing090_pitch15_range3000

    # بدون تقرير HTML
    python run_comprehensive_test.py --no-html

Outputs:
    results/<timestamp>/
        <scenario>_baseline.csv
        <scenario>_result.json
        matrix_summary.csv
        coverage_report.html
"""

from __future__ import annotations
import os
import sys
import argparse
import time
import traceback
from datetime import datetime
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from config_matrix import build_full_matrix, summarise_matrix, Scenario
import scenario_runner as sr
import coverage_report as cov


def parse_args():
    ap = argparse.ArgumentParser(
        description='M130 Comprehensive SITL Test Runner — اختبار موسّع شامل')
    ap.add_argument('--quick', action='store_true',
                    help='مصفوفة مُصغَّرة (~10 سيناريو) للتطوير السريع')
    ap.add_argument('--with-sitl', action='store_true',
                    help='شغِّل subset من السيناريوهات مع PX4 SITL لـ parity check')
    ap.add_argument('--px4-bin', type=str, default=None,
                    help='مسار binary الـ PX4 (مطلوب مع --with-sitl)')
    ap.add_argument('--only', type=str, default=None,
                    help='تشغيل سيناريو واحد بالاسم')
    ap.add_argument('--tag', type=str, default=None,
                    help='تصفية السيناريوهات حسب tag (مثل: bearing_sweep, failure, near_vertical)')
    ap.add_argument('--base-config', type=str, default=None,
                    help='مسار YAML الأساسي (افتراضي: config/6dof_config_advanced.yaml)')
    ap.add_argument('--out', type=str, default=None,
                    help='مجلد المخرجات (افتراضي: results/<timestamp>)')
    ap.add_argument('--no-html', action='store_true',
                    help='تخطّي توليد تقرير HTML')
    ap.add_argument('--list', action='store_true',
                    help='اعرض المصفوفة فقط بدون تشغيل')
    ap.add_argument('--max-duration', type=float, default=60.0,
                    help='الحد الأعلى لمدّة كل سيناريو (ثانية، افتراضي=60)')
    ap.add_argument('--stop-on-error', action='store_true',
                    help='أوقف عند أول سيناريو يفشل')
    ap.add_argument('--quiet', action='store_true', default=True,
                    help='تقليل output تقدّم المحاكاة')
    return ap.parse_args()


def _load_base_cfg(path):
    import yaml
    cfg_path = path or str(_HERE.parent / 'config' / '6dof_config_advanced.yaml')
    with open(cfg_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    args = parse_args()

    print()
    print("╔═══════════════════════════════════════════════════════════════════╗")
    print("║  M130 COMPREHENSIVE SITL TEST — اختبار موسّع يكشف 13 bug معروفاً   ║")
    print("╚═══════════════════════════════════════════════════════════════════╝")

    # ─── bina  matrix ────────────────────────────────────────────────
    all_scenarios = build_full_matrix(
        include_sitl_parity=args.with_sitl,
        quick=args.quick,
    )

    if args.only:
        all_scenarios = [s for s in all_scenarios if s.name == args.only]
        if not all_scenarios:
            print(f"[!] Scenario '{args.only}' not found")
            return 1
    if args.tag:
        all_scenarios = [s for s in all_scenarios if args.tag in s.tags]
        if not all_scenarios:
            print(f"[!] No scenarios matched tag '{args.tag}'")
            return 1

    summary = summarise_matrix(all_scenarios)
    print(f"\n  Total scenarios : {summary['total_scenarios']}")
    print(f"  By tag          : {summary['by_tag']}")
    print(f"  Expected bug detections : {summary['by_expected_detection']}")

    if args.list:
        print()
        for s in all_scenarios:
            print(f"    - {s.name:50s} tags={s.tags}")
        return 0

    # ─── output dir ──────────────────────────────────────────────────
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = args.out or str(_HERE / 'results' / ts)
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n  Output dir      : {out_dir}\n")

    # ─── base config ─────────────────────────────────────────────────
    try:
        base_cfg = _load_base_cfg(args.base_config)
    except Exception as e:
        print(f"[!] Could not load base config: {e}")
        return 2

    # global max duration
    base_cfg.setdefault('simulation', {})['duration'] = min(
        args.max_duration, base_cfg.get('simulation', {}).get('duration', 60.0))

    # ─── run each scenario ───────────────────────────────────────────
    t0 = time.monotonic()
    results = []
    for i, scenario in enumerate(all_scenarios, 1):
        run_sitl = args.with_sitl and ('require_sitl' in scenario.tags or 'parity' in scenario.tags)
        marker = "[SITL]" if run_sitl else "      "
        print(f"  [{i:3d}/{len(all_scenarios)}] {marker} {scenario.name}")

        t_start = time.monotonic()
        try:
            result = sr.run_scenario(
                scenario,
                base_cfg=base_cfg,
                out_dir=out_dir,
                run_sitl=run_sitl,
                px4_bin=args.px4_bin,
                quiet=args.quiet,
            )
            results.append(result)
            elapsed = time.monotonic() - t_start

            if not result.get('baseline_ok'):
                err = result.get('baseline_error', '?')
                print(f"           CRASHED ({elapsed:.1f}s) — {err[:70]}")
                if args.stop_on_error:
                    break
                continue

            ok = result.get('overall_pass', False)
            cep = result.get('metrics', {}).get('impact_cep_m')
            cep_s = f"CEP={cep:.1f}m " if cep is not None else ''
            tag = 'PASS' if ok else 'FAIL'
            detected = result.get('detected_bugs', [])
            det_s = f"detected={detected}" if detected else ''
            print(f"           {tag:4s} ({elapsed:.1f}s) {cep_s}{det_s}")

            if not ok and args.stop_on_error:
                break
        except Exception as e:
            print(f"           EXCEPTION: {type(e).__name__}: {e}")
            traceback.print_exc()
            results.append({
                'scenario': scenario.name,
                'tags': scenario.tags,
                'expected_detects': scenario.expected_detects,
                'baseline_ok': False,
                'baseline_error': str(e),
                'overall_pass': False,
            })
            if args.stop_on_error:
                break

    total_elapsed = time.monotonic() - t0
    print(f"\n  Total wall time: {total_elapsed:.1f}s")

    # ─── aggregate reports ───────────────────────────────────────────
    coverage = cov.compute_coverage(results)
    cov.print_text_summary(coverage)

    summary_csv = os.path.join(out_dir, 'matrix_summary.csv')
    cov.build_summary_csv(results, summary_csv)
    print(f"  Summary CSV  → {summary_csv}")

    if not args.no_html:
        html_path = os.path.join(out_dir, 'coverage_report.html')
        cov.generate_html_report(results, coverage, html_path)
        print(f"  HTML report  → {html_path}")

    # non-zero exit if anything failed
    return 0 if coverage['failed'] == 0 and coverage['crashed'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
