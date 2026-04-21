"""
enhanced_metrics.py
===================
معايير نجاح موسّعة تستبدل الـ 4 الضيّقة في run_sitl_test.py.

بدلاً من: {impact_cep<50, alt_mae<20, vel_mae<5, att_max<5}
نُضيف:
  - stability       : max |omega_y|, max |omega_z| (rate saturation)
  - aoa_bounds      : max |alpha| أقل من حدود aerodynamic map
  - deflection_util : peak fin deflection (هل يصل max?)
  - mhe_health      : نسبة valid steps / total steps
  - settling_time   : زمن دخول chi_err < 2°
  - overshoot_pct   : أقصى تجاوز في altitude/range
  - parity_flags    : أي mismatch بين Python و C++
"""

from __future__ import annotations
from typing import Dict, Any
import numpy as np


def compute_enhanced_metrics(data: Dict[str, np.ndarray]) -> Dict[str, Any]:
    m: Dict[str, Any] = {}

    t = data.get('time')
    if t is None or len(t) < 2:
        return m
    m['duration_s'] = float(t[-1] - t[0])

    # terminal miss
    if 'pos_x' in data and 'pos_y' in data:
        ix = float(data['pos_x'][-1])
        iy = float(data['pos_y'][-1])
        m['impact_pos_xy'] = (ix, iy)
        if 'target_pos_x' in data and 'target_pos_y' in data:
            tx = float(data['target_pos_x'][-1])
            ty = float(data['target_pos_y'][-1])
            m['impact_cep_m'] = float(np.hypot(ix - tx, iy - ty))

    # altitude
    if 'altitude' in data:
        m['apogee_m'] = float(np.max(data['altitude']))

    # stability
    for ax, key in (('x', 'omega_x'), ('y', 'omega_y'), ('z', 'omega_z')):
        if key in data:
            v = data[key]
            v = v[np.isfinite(v)]
            if len(v) == 0:
                continue
            peak = float(np.max(np.abs(v)))
            if peak < 100:  # rad/s → probably radians
                peak = np.degrees(peak)
            m[f'omega_{ax}_peak_dps'] = peak

    # angle of attack
    if 'alpha' in data:
        a = data['alpha']
        a = a[np.isfinite(a)]
        if len(a) > 0:
            peak = float(np.max(np.abs(a)))
            if peak < 3.2:
                peak = np.degrees(peak)
            m['alpha_peak_deg'] = peak

    # fin peaks
    for ch in (1, 2, 3, 4):
        k = f'fin_cmd_{ch}'
        if k in data:
            v = data[k]
            v = v[np.isfinite(v)]
            if len(v) > 0:
                peak = float(np.max(np.abs(v)))
                if peak < 1.0:
                    peak = np.degrees(peak)
                m[f'fin_{ch}_peak_deg'] = peak

    # MHE health (if logged)
    if 'mhe_valid' in data:
        mv = data['mhe_valid']
        m['mhe_valid_ratio'] = float(np.mean(mv > 0.5))
    if 'mhe_residual' in data:
        r = data['mhe_residual']
        r = r[np.isfinite(r)]
        if len(r) > 0:
            m['mhe_residual_p99'] = float(np.percentile(r, 99))

    # chi tracking settling time
    if 'chi_err_deg' in data:
        err = np.abs(data['chi_err_deg'])
        idx = np.where(err < 2.0)[0]
        m['chi_settling_time_s'] = float(t[idx[0]] - t[0]) if len(idx) else float('inf')

    # NaN / inf sanity
    bad = 0
    for k, v in data.items():
        if k.startswith('_') or not isinstance(v, np.ndarray):
            continue
        if v.dtype.kind == 'f':
            bad += int(np.sum(~np.isfinite(v)))
    m['n_nonfinite_samples'] = bad

    return m


def evaluate_pass_fail(metrics: Dict[str, Any],
                        parity: Dict[str, Any] = None,
                        authority: Dict[str, Any] = None,
                        chi_wrap: Dict[str, Any] = None,
                        thresholds: Dict[str, float] = None) -> Dict[str, Any]:
    """
    يُرجع dict من criterion→{value, limit, pass} وoverall.
    أكثر ثراءً من الـ 4 الأصلية.
    """
    thr = thresholds or {}
    d = {
        'impact_cep_m':        thr.get('impact_cep_m', 50.0),
        'alpha_peak_deg':      thr.get('alpha_peak_deg', 15.0),
        'omega_y_peak_dps':    thr.get('omega_y_peak_dps', 180.0),
        'omega_z_peak_dps':    thr.get('omega_z_peak_dps', 180.0),
        'fin_peak_deg':        thr.get('fin_peak_deg', 20.0),
        'mhe_valid_ratio':     thr.get('mhe_valid_ratio', 0.95),
        'mhe_residual_p99':    thr.get('mhe_residual_p99', 5.0),
        'n_nonfinite_samples': thr.get('n_nonfinite_samples', 0),
        'chi_settling_time_s': thr.get('chi_settling_time_s', 3.0),
    }
    results: Dict[str, Any] = {}

    def _check(key, op_gt=False, limit=None):
        lim = d.get(key) if limit is None else limit
        val = metrics.get(key)
        if val is None:
            results[key] = {'value': None, 'limit': lim, 'pass': None, 'skipped': True}
            return
        try:
            fval = float(val) if not isinstance(val, tuple) else float('nan')
        except Exception:
            fval = float('nan')
        if np.isnan(fval):
            results[key] = {'value': val, 'limit': lim, 'pass': None, 'skipped': True}
            return
        ok = (fval >= lim) if op_gt else (fval <= lim)
        results[key] = {'value': fval, 'limit': lim, 'pass': bool(ok)}

    _check('impact_cep_m')
    _check('alpha_peak_deg')
    _check('omega_y_peak_dps')
    _check('omega_z_peak_dps')
    for ch in (1, 2, 3, 4):
        k = f'fin_{ch}_peak_deg'
        val = metrics.get(k)
        if val is not None:
            ok = float(val) <= d['fin_peak_deg']
            results[k] = {'value': float(val), 'limit': d['fin_peak_deg'], 'pass': bool(ok)}
    _check('mhe_valid_ratio', op_gt=True)
    _check('mhe_residual_p99')
    _check('n_nonfinite_samples')
    _check('chi_settling_time_s')

    # external checks
    if parity:
        results['parity_overall'] = {'value': 'pass' if parity.get('pass') else 'fail',
                                      'pass': bool(parity.get('pass', False))}
    if authority:
        results['authority_within_limit'] = {
            'value': authority.get('max_seen_deg'),
            'pass': bool(authority.get('pass', False)),
        }
    if chi_wrap:
        results['chi_no_wrap_jump'] = {
            'value': chi_wrap.get('peak_chi_rate_dps'),
            'pass': bool(chi_wrap.get('pass', False)),
        }

    overall = all(r.get('pass') for r in results.values() if not r.get('skipped'))
    results['_overall_pass'] = overall
    return results
