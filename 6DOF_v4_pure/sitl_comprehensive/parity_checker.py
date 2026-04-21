"""
parity_checker.py
=================
يقارن سلسلة زمنية بين مصدرين:
  - baseline (Python MPC)  vs  SITL (C++ rocket_mpc)
  - MHE estimate           vs  ground truth من sim
  - body-frame accel (IMU) vs  expected specific force

يكشف:
  BUG-X1  → body_accel_mhe - body_accel_sim ≈ g (طرح مزدوج للجاذبية)
  BUG-G   → MHE alpha/V يختلف عند V<5 m/s
  BUG-1/2 → MPC Python يُنتج fin_cmd مختلف عن C++
"""

from __future__ import annotations
from typing import Dict, Any, List, Tuple
import numpy as np


def _interp(t_ref: np.ndarray, t: np.ndarray, y: np.ndarray) -> np.ndarray:
    if len(t) < 2:
        return np.zeros_like(t_ref)
    return np.interp(t_ref, t, y)


def check_mhe_vs_truth(baseline_csv_data: Dict[str, np.ndarray],
                       tol: Dict[str, float] = None) -> Dict[str, Any]:
    """
    baseline CSV يحوي كلاً من الحالة الحقيقية وتقدير MHE (إذا كان mode='mhe').
    المفاتيح المتوقعة:
        ground truth: vel_x, vel_y, vel_z, alpha, gamma_deg
        MHE estimate: mhe_vel_x, mhe_vel_y, mhe_vel_z, mhe_alpha, mhe_gamma_deg
                      (قد لا تُصدَّر افتراضياً — نكتشف ذلك)
    """
    tol = tol or {
        'velocity_rms_ms': 2.0,
        'alpha_rms_deg':   1.5,
        'gamma_rms_deg':   1.5,
        'accel_bias_ms2':  1.0,
    }
    out: Dict[str, Any] = {'checks': {}, 'pass': True, 'missing': []}
    d = baseline_csv_data

    t = d.get('time')
    if t is None:
        out['pass'] = False
        out['missing'].append('time')
        return out

    # حقن نافذة زمنية مفيدة (بعد launch، قبل apogee)
    if 'altitude' in d:
        mask = (t > 0.5) & (d['altitude'] > 5.0)
    else:
        mask = t > 0.5
    if not mask.any():
        out['pass'] = False
        out['missing'].append('no_valid_window')
        return out

    def _rms(a, b):
        e = a[mask] - b[mask]
        e = e[np.isfinite(e)]
        return float(np.sqrt(np.mean(e**2))) if len(e) else float('nan')

    # velocity
    if all(k in d for k in ('vel_x', 'vel_y', 'vel_z',
                             'mhe_vel_x', 'mhe_vel_y', 'mhe_vel_z')):
        vx = _rms(d['mhe_vel_x'], d['vel_x'])
        vy = _rms(d['mhe_vel_y'], d['vel_y'])
        vz = _rms(d['mhe_vel_z'], d['vel_z'])
        v_total = float(np.sqrt(vx**2 + vy**2 + vz**2))
        ok = v_total < tol['velocity_rms_ms']
        out['checks']['mhe_velocity_rms_ms'] = {'value': v_total, 'tol': tol['velocity_rms_ms'], 'pass': ok}
        out['pass'] &= ok
    else:
        out['missing'].append('mhe_velocity_columns')

    # alpha
    if 'alpha' in d and 'mhe_alpha' in d:
        e = _rms(np.degrees(d['mhe_alpha']), np.degrees(d['alpha']))
        ok = e < tol['alpha_rms_deg']
        out['checks']['mhe_alpha_rms_deg'] = {'value': e, 'tol': tol['alpha_rms_deg'], 'pass': ok}
        out['pass'] &= ok
    else:
        out['missing'].append('mhe_alpha')

    # gamma (flight path angle)
    if 'gamma_deg' in d and 'mhe_gamma_deg' in d:
        e = _rms(d['mhe_gamma_deg'], d['gamma_deg'])
        ok = e < tol['gamma_rms_deg']
        out['checks']['mhe_gamma_rms_deg'] = {'value': e, 'tol': tol['gamma_rms_deg'], 'pass': ok}
        out['pass'] &= ok
    else:
        out['missing'].append('mhe_gamma')

    # body-frame accel bias (يكشف BUG-X1 مباشرة)
    if all(k in d for k in ('accel_body_x', 'accel_body_y', 'accel_body_z',
                             'mhe_accel_body_x', 'mhe_accel_body_y', 'mhe_accel_body_z')):
        bx = float(np.nanmean(d['mhe_accel_body_x'][mask] - d['accel_body_x'][mask]))
        by = float(np.nanmean(d['mhe_accel_body_y'][mask] - d['accel_body_y'][mask]))
        bz = float(np.nanmean(d['mhe_accel_body_z'][mask] - d['accel_body_z'][mask]))
        mag = float(np.sqrt(bx**2 + by**2 + bz**2))
        ok = mag < tol['accel_bias_ms2']
        out['checks']['accel_body_bias_ms2'] = {
            'value': mag, 'bias_xyz': (bx, by, bz),
            'tol': tol['accel_bias_ms2'], 'pass': ok,
            'hint': 'if ~9.8 m/s² → BUG-X1 (double gravity subtraction)'
        }
        out['pass'] &= ok
    else:
        out['missing'].append('accel_body_columns')

    return out


def check_python_vs_cpp(baseline: Dict[str, np.ndarray],
                         sitl: Dict[str, np.ndarray],
                         tol: Dict[str, float] = None) -> Dict[str, Any]:
    """Parity check لـ fin commands وattitude بين Python MPC و C++ MPC."""
    tol = tol or {
        'fin_rms_deg':      2.0,
        'attitude_rms_deg': 3.0,
        'impact_cep_m':    30.0,
    }
    out: Dict[str, Any] = {'checks': {}, 'pass': True, 'missing': []}

    tb = baseline.get('time')
    ts = sitl.get('time')
    if tb is None or ts is None:
        out['pass'] = False
        out['missing'].append('time')
        return out

    t_common = np.linspace(max(tb[0], ts[0]),
                            min(tb[-1], ts[-1]),
                            min(2000, min(len(tb), len(ts))))
    if len(t_common) < 10:
        out['pass'] = False
        out['missing'].append('no_common_window')
        return out

    # fin commands RMS
    for ch in (1, 2, 3, 4):
        kb = f'fin_cmd_{ch}'
        ks = f'fin_cmd_{ch}'
        if kb in baseline and ks in sitl:
            a = np.degrees(_interp(t_common, tb, baseline[kb]))
            b = np.degrees(_interp(t_common, ts, sitl[ks]))
            rms = float(np.sqrt(np.mean((a - b)**2)))
            ok = rms < tol['fin_rms_deg']
            out['checks'][f'fin_{ch}_rms_deg'] = {'value': rms, 'tol': tol['fin_rms_deg'], 'pass': ok}
            out['pass'] &= ok

    # attitude RMS (via quat → small-angle)
    if all(k in baseline for k in ('q0', 'q1', 'q2', 'q3')) and \
       all(k in sitl     for k in ('q0', 'q1', 'q2', 'q3')):
        qb = np.stack([_interp(t_common, tb, baseline[f'q{i}']) for i in range(4)], axis=1)
        qs = np.stack([_interp(t_common, ts, sitl[f'q{i}']) for i in range(4)], axis=1)
        dot = np.clip(np.sum(qb * qs, axis=1), -1.0, 1.0)
        ang = 2.0 * np.degrees(np.arccos(np.abs(dot)))
        rms = float(np.sqrt(np.mean(ang**2)))
        ok = rms < tol['attitude_rms_deg']
        out['checks']['attitude_rms_deg'] = {'value': rms, 'tol': tol['attitude_rms_deg'], 'pass': ok}
        out['pass'] &= ok

    # impact CEP
    for src, data in (('baseline', baseline), ('sitl', sitl)):
        if 'pos_x' in data and 'pos_y' in data:
            out[f'{src}_impact'] = (float(data['pos_x'][-1]), float(data['pos_y'][-1]))
    if 'baseline_impact' in out and 'sitl_impact' in out:
        dx = out['sitl_impact'][0] - out['baseline_impact'][0]
        dy = out['sitl_impact'][1] - out['baseline_impact'][1]
        cep = float(np.hypot(dx, dy))
        ok = cep < tol['impact_cep_m']
        out['checks']['impact_cep_m'] = {'value': cep, 'tol': tol['impact_cep_m'], 'pass': ok}
        out['pass'] &= ok

    return out


def check_control_authority(baseline: Dict[str, np.ndarray],
                             max_deflection_deg: float = 20.0,
                             margin_deg: float = 1.0) -> Dict[str, Any]:
    """
    يفحص ما إذا كان الـ solver يستخدم الـ max deflection المُعلَن.
    إذا الحقيقي أعلى من max_deflection_deg → عدم تطابق constraint (BUG-E).
    """
    out: Dict[str, Any] = {'checks': {}, 'pass': True}
    peaks = []
    for ch in (1, 2, 3, 4):
        k = f'fin_cmd_{ch}'
        if k in baseline:
            v = baseline[k]
            v = v[np.isfinite(v)]
            if len(v) == 0:
                continue
            peak = float(np.max(np.abs(np.degrees(v))))
            peaks.append((ch, peak))
    out['peaks_deg'] = peaks
    if peaks:
        max_seen = max(p for _, p in peaks)
        out['max_seen_deg'] = max_seen
        over = max_seen > (max_deflection_deg + margin_deg)
        out['checks']['deflection_within_limit'] = {
            'max_seen': max_seen, 'limit': max_deflection_deg,
            'pass': not over,
            'hint': 'if exceeds → BUG-E (solver uses 20° but param declares 25°)',
        }
        out['pass'] = not over
    return out


def check_chi_wrap(baseline: Dict[str, np.ndarray],
                    max_rate_jump_dps: float = 500.0) -> Dict[str, Any]:
    """
    يكشف angle-wrap في chi: قفزة ±360° في أقل من خطوة واحدة.
    """
    out: Dict[str, Any] = {'pass': True}
    if 'chi_cmd_deg' not in baseline or 'time' not in baseline:
        out['missing'] = 'chi_cmd_deg'
        return out
    t = baseline['time']
    chi = baseline['chi_cmd_deg']
    dt = np.diff(t)
    dchi = np.diff(chi)
    dt = np.where(dt > 1e-6, dt, 1e-6)
    rate = np.abs(dchi / dt)
    peak = float(np.max(rate)) if len(rate) else 0.0
    out['peak_chi_rate_dps'] = peak
    out['pass'] = peak < max_rate_jump_dps
    out['hint'] = 'if > 500°/s → BUG-H (no angle wrap in rate limiter)'
    return out
