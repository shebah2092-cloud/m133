"""
scenario_runner.py
==================
يشغّل سيناريو واحد:
  1. يأخذ base config + overrides
  2. يكتبه إلى temp YAML
  3. يشغّل Python baseline (اختيارياً + SITL)
  4. يُنتج CSV + metrics + parity report

يُعزل في process منفصل أحياناً لتفادي global state في sim.
"""

from __future__ import annotations
import os
import sys
import time
import yaml
import json
import tempfile
import traceback
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
import numpy as np

_HERE = Path(__file__).resolve().parent
_SIM_DIR = _HERE.parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_HERE))

from config_matrix import Scenario, apply_overrides
from failure_injector import FailureInjector
import enhanced_metrics as em
import parity_checker as pc


def _load_base_config(path: Optional[str] = None) -> Dict[str, Any]:
    cfg_path = path or str(_SIM_DIR / 'config' / '6dof_config_advanced.yaml')
    with open(cfg_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _write_temp_config(cfg: Dict[str, Any]) -> str:
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                       delete=False, encoding='utf-8')
    yaml.dump(cfg, tmp, allow_unicode=True, default_flow_style=False)
    tmp.close()
    return tmp.name


def _load_csv(path: str) -> Dict[str, np.ndarray]:
    import csv
    raw: Dict[str, list] = {}
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for k in reader.fieldnames:
            raw[k] = []
        for row in reader:
            for k in reader.fieldnames:
                try:
                    raw[k].append(float(row[k]))
                except (ValueError, TypeError):
                    raw[k].append(float('nan'))
    data = {k: np.array(v) for k, v in raw.items()}

    # normalise common alias names
    ALIAS = {
        'time': ['time_s'],
        'vel_x': ['velocity_x_m_s'], 'vel_y': ['velocity_y_m_s'], 'vel_z': ['velocity_z_m_s'],
        'pos_x': ['position_x_m'],   'pos_y': ['position_y_m'],   'pos_z': ['position_z_m'],
        'altitude': ['altitude_m'],
        'alpha': ['alpha_rad'], 'beta': ['beta_rad'],
        'q0': ['quat_w'], 'q1': ['quat_x'], 'q2': ['quat_y'], 'q3': ['quat_z'],
        'omega_x': ['omega_x_rad_s'], 'omega_y': ['omega_y_rad_s'], 'omega_z': ['omega_z_rad_s'],
        'fin_cmd_1': ['fin_1_rad'], 'fin_cmd_2': ['fin_2_rad'],
        'fin_cmd_3': ['fin_3_rad'], 'fin_cmd_4': ['fin_4_rad'],
    }
    for canonical, cands in ALIAS.items():
        if canonical in data:
            continue
        for c in cands:
            if c in data:
                data[canonical] = data[c]
                break
    return data


def run_python_baseline(cfg: Dict[str, Any],
                        out_csv: str,
                        on_progress=None) -> Dict[str, Any]:
    """يشغّل Python sim بالـ config المُعطى."""
    from rocket_6dof_sim import Rocket6DOFSimulation, export_comprehensive_log

    # ضمان تفعيل MPC
    cfg.setdefault('simulation', {})['control_type'] = 'mpc'

    tmp_path = _write_temp_config(cfg)
    try:
        long_range = cfg.get('long_range', {}).get('enabled', False)
        sim = Rocket6DOFSimulation(config_file=tmp_path, long_range_mode=long_range)

        duration = cfg.get('simulation', {}).get('duration', 60.0)
        dt = cfg.get('simulation', {}).get('dt', 0.01)

        t0 = time.monotonic()
        history = sim.simulate(duration=duration, dt=dt,
                                on_step=on_progress, callback_stride=100)
        wall = time.monotonic() - t0

        os.makedirs(os.path.dirname(out_csv), exist_ok=True)
        export_comprehensive_log(history, sim, out_csv)

        return {
            'wall_time_s': wall,
            'n_points': len(history.get('time', [])),
            't_final': float(history['time'][-1]) if history.get('time') else 0.0,
            'csv': out_csv,
        }
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


def run_scenario(scenario: Scenario,
                  base_cfg: Dict[str, Any],
                  out_dir: str,
                  run_sitl: bool = False,
                  px4_bin: Optional[str] = None,
                  quiet: bool = True) -> Dict[str, Any]:
    """
    يشغّل سيناريو → يُنتج:
      out_dir/<scenario>.csv
      out_dir/<scenario>_metrics.json
    """
    os.makedirs(out_dir, exist_ok=True)
    cfg = apply_overrides(base_cfg, scenario.overrides)

    # مدّة افتراضية أقصر للـ sweep (60s يكفي لمعظم الـ impact cases)
    dur_default = 60.0
    if 'simulation' not in cfg:
        cfg['simulation'] = {}
    if scenario.duration_override:
        cfg['simulation']['duration'] = float(scenario.duration_override)
    else:
        cfg['simulation'].setdefault('duration', dur_default)
        cfg['simulation']['duration'] = min(cfg['simulation']['duration'], dur_default)

    baseline_csv = os.path.join(out_dir, f"{scenario.name}_baseline.csv")
    result: Dict[str, Any] = {
        'scenario': scenario.name,
        'tags': scenario.tags,
        'expected_detects': scenario.expected_detects,
        'overrides': scenario.overrides,
        'failure_type': scenario.failure_type,
        'started_at': time.time(),
    }

    # ─── Python baseline ─────────────────────────────────────────────
    try:
        def prog(info):
            if quiet:
                return
            print(f"\r    [{scenario.name}] {info['progress']:5.1f}% "
                  f"alt={info['altitude']/1000:.2f}km phase={info['flight_phase']}",
                  end='', flush=True)

        run_info = run_python_baseline(cfg, baseline_csv, on_progress=prog)
        if not quiet:
            print()
        result['baseline'] = run_info
        result['baseline_ok'] = True
    except Exception as e:
        result['baseline_ok'] = False
        result['baseline_error'] = f"{type(e).__name__}: {e}"
        result['baseline_traceback'] = traceback.format_exc()
        # لا نكمل بقية الفحوصات لو الـ baseline انهار
        _write_result(result, out_dir, scenario.name)
        return result

    # ─── تحليل baseline CSV ──────────────────────────────────────────
    try:
        base_data = _load_csv(baseline_csv)
        metrics = em.compute_enhanced_metrics(base_data)
        result['metrics'] = metrics

        authority = pc.check_control_authority(base_data)
        chi_wrap = pc.check_chi_wrap(base_data)
        mhe_truth = pc.check_mhe_vs_truth(base_data)

        result['check_control_authority'] = authority
        result['check_chi_wrap'] = chi_wrap
        result['check_mhe_vs_truth'] = mhe_truth

        # ─── optional SITL ───────────────────────────────────────────
        parity = None
        if run_sitl:
            sitl_csv = os.path.join(out_dir, f"{scenario.name}_sitl.csv")
            try:
                sitl_info = run_sitl_scenario(cfg, sitl_csv, px4_bin)
                result['sitl'] = sitl_info
                sitl_data = _load_csv(sitl_csv)
                parity = pc.check_python_vs_cpp(base_data, sitl_data)
                result['check_python_vs_cpp'] = parity
            except Exception as e:
                result['sitl_error'] = f"{type(e).__name__}: {e}"

        # ─── pass/fail ───────────────────────────────────────────────
        eval_out = em.evaluate_pass_fail(
            metrics=metrics,
            parity=parity,
            authority=authority,
            chi_wrap=chi_wrap,
        )
        result['evaluation'] = eval_out
        result['overall_pass'] = bool(eval_out.get('_overall_pass', False))

        # ─── detected bugs ───────────────────────────────────────────
        detected = []
        bias = mhe_truth.get('checks', {}).get('accel_body_bias_ms2')
        if bias and bias.get('value', 0) > 5.0:
            detected.append('BUG-X1')
        if not chi_wrap.get('pass', True):
            detected.append('BUG-H')
        if not authority.get('pass', True):
            detected.append('BUG-E')
        if metrics.get('n_nonfinite_samples', 0) > 0 and scenario.failure_type == 'nan_accel':
            detected.append('BUG-3')
        result['detected_bugs'] = detected

    except Exception as e:
        result['analysis_error'] = f"{type(e).__name__}: {e}"
        result['analysis_traceback'] = traceback.format_exc()

    result['elapsed_s'] = time.time() - result['started_at']
    _write_result(result, out_dir, scenario.name)
    return result


def run_sitl_scenario(cfg: Dict[str, Any], out_csv: str,
                       px4_bin: Optional[str] = None) -> Dict[str, Any]:
    """تشغيل SITL عبر mavlink_bridge الأصلي (بـ config مُعدَّل)."""
    # نستعمل الـ bridge الأصلي من sitl/ كـ library دون تعديل
    import importlib.util
    bridge_path = _SIM_DIR / 'sitl' / 'mavlink_bridge.py'
    spec = importlib.util.spec_from_file_location('mavlink_bridge_orig', bridge_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    tmp_cfg = _write_temp_config(cfg)
    try:
        bridge = mod.SITLBridge(tmp_cfg, tmp_cfg)
        bridge.run(csv_output=out_csv)
    finally:
        try:
            os.unlink(tmp_cfg)
        except OSError:
            pass
    return {'csv': out_csv}


def _write_result(result: Dict[str, Any], out_dir: str, name: str) -> None:
    def _conv(o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return float(o)
        if isinstance(o, (set, tuple)):
            return list(o)
        return str(o)

    path = os.path.join(out_dir, f"{name}_result.json")
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(result, f, default=_conv, indent=2)
