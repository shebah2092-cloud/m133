"""
coverage_report.py
==================
يجمع نتائج كل السيناريوهات ويُنتج:
  - matrix_summary.csv
  - coverage_report.html  (تفاعلي)
  - text summary إلى stdout

التقرير يُظهر:
  - كم سيناريو نجح / فشل
  - أي bugs اكتُشفت فعلياً (detected_bugs)
  - أي bugs كان متوقَّع كشفها ولم تُكشَف (missed detections)
  - أي سيناريو نجح لكن المتوقع أن يفشل (false negative)
"""

from __future__ import annotations
import os
import json
import csv
from datetime import datetime
from typing import List, Dict, Any
from html import escape as html_escape

KNOWN_BUGS = {
    'BUG-A':  'NED vs bearing-frame mixing in ekf_beta (RocketMPC.cpp:753)',
    'BUG-B':  'chi_0 from bearing not yaw (RocketMPC.cpp:518)',
    'BUG-C':  '_launch_alt_captured not reset on disarm',
    'BUG-D':  'MHE repeats last measurement in stages 10..19',
    'BUG-E':  'MAX_DEFL param=25° but solver baked=20°',
    'BUG-F':  'no staleness check on lpos/att in Run()',
    'BUG-G':  'MHE missing aero_fade at V<5 m/s',
    'BUG-H':  'chi rate-limiter without angle wrap',
    'BUG-X1': 'double gravity subtraction (mavlink_bridge.py:447)',
    'BUG-X2': 'launch threshold fails at pitch>=88°',
    'BUG-1':  'gamma_err = |gamma| not |gamma - gamma_ref|',
    'BUG-2':  'cp1 = cp0 + 0.15 instead of fminf(cp0+0.10, 0.95)',
    'BUG-3':  'degraded.valid stays true through MHE fail cascade',
}


def load_results(out_dir: str) -> List[Dict[str, Any]]:
    results = []
    for fn in sorted(os.listdir(out_dir)):
        if fn.endswith('_result.json'):
            with open(os.path.join(out_dir, fn), 'r', encoding='utf-8') as f:
                try:
                    results.append(json.load(f))
                except json.JSONDecodeError:
                    pass
    return results


def build_summary_csv(results: List[Dict[str, Any]], path: str) -> None:
    rows = []
    for r in results:
        m = r.get('metrics', {})
        rows.append({
            'scenario':        r.get('scenario'),
            'tags':            ','.join(r.get('tags', [])),
            'overall_pass':    r.get('overall_pass'),
            'baseline_ok':     r.get('baseline_ok'),
            'elapsed_s':       round(r.get('elapsed_s', 0), 2),
            'impact_cep_m':    round(m.get('impact_cep_m', float('nan')), 2) if m.get('impact_cep_m') is not None else '',
            'apogee_m':        round(m.get('apogee_m', 0), 1),
            'alpha_peak_deg':  round(m.get('alpha_peak_deg', 0), 2),
            'omega_y_peak':    round(m.get('omega_y_peak_dps', 0), 1),
            'fin_peak_1':      round(m.get('fin_1_peak_deg', 0), 2),
            'chi_settling_s':  m.get('chi_settling_time_s', ''),
            'n_nonfinite':     m.get('n_nonfinite_samples', 0),
            'mhe_valid_ratio': m.get('mhe_valid_ratio', ''),
            'detected_bugs':   ','.join(r.get('detected_bugs', [])),
            'expected_detects': ','.join(r.get('expected_detects', [])),
            'error':           r.get('baseline_error') or r.get('analysis_error') or '',
        })
    if not rows:
        return
    with open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def compute_coverage(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    total = len(results)
    passed = sum(1 for r in results if r.get('overall_pass'))
    failed = total - passed
    crashed = sum(1 for r in results if not r.get('baseline_ok'))

    # bug coverage
    all_expected: Dict[str, int] = {}
    all_detected: Dict[str, int] = {}
    for r in results:
        for b in r.get('expected_detects', []):
            all_expected[b] = all_expected.get(b, 0) + 1
        for b in r.get('detected_bugs', []):
            all_detected[b] = all_detected.get(b, 0) + 1

    missed = {b: all_expected[b] for b in all_expected if b not in all_detected}
    bug_coverage = {
        bug: {
            'description': KNOWN_BUGS.get(bug, ''),
            'expected_in_n_scenarios': all_expected.get(bug, 0),
            'actually_detected_in_n':  all_detected.get(bug, 0),
            'status': 'detected' if bug in all_detected else ('missed' if bug in all_expected else 'not_tested'),
        }
        for bug in KNOWN_BUGS
    }

    return {
        'total_scenarios': total,
        'passed': passed,
        'failed': failed,
        'crashed': crashed,
        'pass_rate_pct': round(100.0 * passed / total, 1) if total else 0.0,
        'bug_coverage': bug_coverage,
        'missed_detections': missed,
    }


def generate_html_report(results: List[Dict[str, Any]],
                          coverage: Dict[str, Any],
                          out_path: str) -> None:
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # summary cards
    cards = f"""
    <div class="grid-4">
      <div class="card metric"><div class="v">{coverage['total_scenarios']}</div><div class="l">Scenarios</div></div>
      <div class="card metric pass"><div class="v">{coverage['passed']}</div><div class="l">Passed</div></div>
      <div class="card metric fail"><div class="v">{coverage['failed']}</div><div class="l">Failed</div></div>
      <div class="card metric warn"><div class="v">{coverage['pass_rate_pct']}%</div><div class="l">Pass rate</div></div>
    </div>"""

    # bug coverage table
    bug_rows = ''
    for bug, info in coverage['bug_coverage'].items():
        status = info['status']
        cls = {'detected': 'pass', 'missed': 'fail', 'not_tested': 'warn'}[status]
        bug_rows += (
            f"<tr><td><b>{bug}</b></td>"
            f"<td>{html_escape(info['description'])}</td>"
            f"<td>{info['expected_in_n_scenarios']}</td>"
            f"<td>{info['actually_detected_in_n']}</td>"
            f"<td><span class='badge {cls}'>{status}</span></td></tr>"
        )

    # scenarios table
    sc_rows = ''
    for r in results:
        m = r.get('metrics', {})
        ok = r.get('overall_pass')
        badge = f"<span class='badge {'pass' if ok else 'fail'}'>{'PASS' if ok else 'FAIL'}</span>"
        cep = m.get('impact_cep_m')
        cep_s = f"{cep:.1f}" if cep is not None else '—'
        detected = ','.join(r.get('detected_bugs', [])) or '—'
        expected = ','.join(r.get('expected_detects', [])) or '—'
        err = r.get('baseline_error') or r.get('analysis_error') or ''
        sc_rows += (
            f"<tr><td>{html_escape(r.get('scenario', ''))}</td>"
            f"<td>{html_escape(','.join(r.get('tags', [])))}</td>"
            f"<td>{badge}</td>"
            f"<td class='mono'>{cep_s}</td>"
            f"<td class='mono'>{m.get('apogee_m', 0):.0f}</td>"
            f"<td class='mono'>{m.get('alpha_peak_deg', 0):.1f}</td>"
            f"<td>{html_escape(expected)}</td>"
            f"<td>{html_escape(detected)}</td>"
            f"<td class='err'>{html_escape(err[:80])}</td></tr>"
        )

    css = """
:root{--bg:#0f172a;--card:#1e293b;--border:#334155;--text:#e2e8f0;--muted:#94a3b8;
--pass:#22c55e;--fail:#ef4444;--warn:#f59e0b;--accent:#3b82f6}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);line-height:1.5;padding:20px}
h1{color:#f8fafc;margin-bottom:6px}.sub{color:var(--muted);margin-bottom:18px}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;margin:12px 0}
.grid-4{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}
.metric{text-align:center}.metric .v{font-size:2em;font-weight:700;color:var(--accent)}
.metric .l{color:var(--muted);font-size:.8em;text-transform:uppercase}
.metric.pass .v{color:var(--pass)}.metric.fail .v{color:var(--fail)}.metric.warn .v{color:var(--warn)}
h2{color:var(--accent);margin:20px 0 10px;padding-bottom:6px;border-bottom:1px solid var(--border)}
table{width:100%;border-collapse:collapse;font-size:.85em}
th{background:#334155;padding:8px 10px;text-align:left;border-bottom:2px solid var(--border)}
td{padding:6px 10px;border-bottom:1px solid var(--border);vertical-align:top}
tr:hover{background:#273449}
.mono{font-family:'Consolas','Fira Code',monospace}
.badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:.75em;font-weight:700;color:#fff}
.badge.pass{background:var(--pass)}.badge.fail{background:var(--fail)}.badge.warn{background:var(--warn)}
.err{color:#fca5a5;font-size:.75em;font-family:monospace}
"""

    html = f"""<!DOCTYPE html><html><head><meta charset="UTF-8">
<title>Comprehensive SITL Coverage Report</title><style>{css}</style></head><body>
<h1>Comprehensive SITL Test Report</h1>
<p class="sub">Generated {now} · {coverage['total_scenarios']} scenarios · 13 known bugs tracked</p>
{cards}

<h2>Known-Bug Coverage Matrix</h2>
<div class="card"><table>
<thead><tr><th>Bug ID</th><th>Description</th><th>Expected in</th><th>Actually detected</th><th>Status</th></tr></thead>
<tbody>{bug_rows}</tbody>
</table></div>

<h2>Scenarios</h2>
<div class="card"><table>
<thead><tr><th>Name</th><th>Tags</th><th>Result</th><th>CEP (m)</th>
<th>Apogee (m)</th><th>α peak (°)</th><th>Expected bugs</th><th>Detected bugs</th><th>Error</th></tr></thead>
<tbody>{sc_rows}</tbody>
</table></div>

</body></html>"""

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)


def print_text_summary(coverage: Dict[str, Any]) -> None:
    print()
    print("=" * 70)
    print("  COMPREHENSIVE SITL TEST — COVERAGE SUMMARY")
    print("=" * 70)
    print(f"  Scenarios run    : {coverage['total_scenarios']}")
    print(f"  Passed           : {coverage['passed']}")
    print(f"  Failed           : {coverage['failed']}")
    print(f"  Crashed          : {coverage['crashed']}")
    print(f"  Pass rate        : {coverage['pass_rate_pct']}%")
    print()
    print("  Known-bug coverage:")
    print("  " + "-" * 66)
    for bug, info in coverage['bug_coverage'].items():
        status = info['status']
        symbol = {'detected': '[+]', 'missed': '[!]', 'not_tested': '[ ]'}[status]
        print(f"    {symbol} {bug:6s} expected={info['expected_in_n_scenarios']:2d} "
              f"detected={info['actually_detected_in_n']:2d} — {info['description'][:45]}")
    print("  " + "-" * 66)
    if coverage['missed_detections']:
        print()
        print("  [!] MISSED DETECTIONS — تحتاج اختبارات أعمق:")
        for b, n in coverage['missed_detections'].items():
            print(f"      - {b} (متوقَّع في {n} سيناريو لكن لم يُكتشف)")
    print()
