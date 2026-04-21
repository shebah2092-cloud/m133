#!/usr/bin/env python3
"""
SITL Test Runner — Runs Python baseline + SITL bridge and generates comparison report.
سكربت اختبار SITL — يشغّل محاكاة Python المرجعية ثم جسر SITL ويولّد تقرير المقارنة.

Usage:
    python run_sitl_test.py                          # Full test (baseline + SITL + compare)
    python run_sitl_test.py --baseline-only           # Run only Python baseline
    python run_sitl_test.py --sitl-only               # Run only SITL (assumes baseline CSV exists)
    python run_sitl_test.py --compare-only            # Compare existing CSVs
    python run_sitl_test.py --px4-bin /path/to/px4    # Specify PX4 binary
"""

import sys
import os
import time
import argparse
import subprocess
import signal
import csv
import json
from pathlib import Path
from datetime import datetime
from html import escape as html_escape

import numpy as np

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    import plotly.io as pio
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS_DIR = _SCRIPT_DIR / 'results'

sys.path.insert(0, str(_SIM_DIR))


# ─── Quaternion → Euler helper ───────────────────────────────────────────────

def _quat_to_euler(q0, q1, q2, q3):
    """Convert quaternion (scalar-first) to Euler angles (roll, pitch, yaw) in degrees."""
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1**2 + q2**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2**2 + q3**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def run_baseline(sim_config: str = None, output_csv: str = None) -> str:
    """Run Python 6DOF simulation with internal MPC as baseline reference.
    
    Returns path to CSV output file.
    """
    print("=" * 70)
    print("  STEP 1: Running Python baseline simulation (MPC internal)")
    print("=" * 70)

    from rocket_6dof_sim import Rocket6DOFSimulation, export_comprehensive_log
    import yaml

    # Load config
    cfg_path = sim_config or str(_SIM_DIR / 'config' / '6dof_config_advanced.yaml')
    with open(cfg_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # Ensure MPC is enabled
    config['simulation']['control_type'] = 'mpc'

    # Write temp config
    import tempfile
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, encoding='utf-8')
    yaml.dump(config, tmp, allow_unicode=True, default_flow_style=False)
    tmp.close()

    long_range = config.get('long_range', {}).get('enabled', False)
    sim = Rocket6DOFSimulation(config_file=tmp.name, long_range_mode=long_range)

    duration = config.get('simulation', {}).get('duration', 500.0)
    dt = config.get('simulation', {}).get('dt', 0.01)

    t_start = time.monotonic()

    def progress(info):
        print(f"\r  [Baseline] {info['progress']:5.1f}% | t={info['time']:6.1f}s | "
              f"alt={info['altitude']/1000:.1f}km | vel={info['velocity']:.0f}m/s | "
              f"phase={info['flight_phase']}", end='', flush=True)

    history = sim.simulate(duration=duration, dt=dt, on_step=progress,
                           callback_stride=100)
    print()

    elapsed = time.monotonic() - t_start
    n_pts = len(history['time'])
    final_t = history['time'][-1] if n_pts > 0 else 0
    print(f"  Baseline complete: {n_pts} points, t_final={final_t:.3f}s, "
          f"wall={elapsed:.1f}s")

    # Export CSV
    csv_path = output_csv or str(_RESULTS_DIR / 'baseline_flight.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    export_comprehensive_log(history, sim, csv_path)
    print(f"  Saved: {csv_path}")

    os.unlink(tmp.name)
    return csv_path


def run_sitl(px4_bin: str = None, sitl_config: str = None,
             sim_config: str = None, output_csv: str = None,
             port: int = 4560) -> str:
    """Run SITL bridge + PX4 binary.
    
    Returns path to CSV output file.
    """
    print()
    print("=" * 70)
    print("  STEP 2: Running SITL test (PX4 rocket_mpc + 6DOF bridge)")
    print("=" * 70)

    csv_path = output_csv or str(_RESULTS_DIR / 'sitl_flight.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    sitl_cfg = sitl_config or str(_SCRIPT_DIR / 'sitl_config.yaml')

    if px4_bin and os.path.isfile(px4_bin):
        # Launch PX4 binary in background
        print(f"  Starting PX4: {px4_bin}")
        px4_env = os.environ.copy()
        px4_env['PX4_SYS_AUTOSTART'] = '22003'  # SITL airframe
        px4_env['PX4_SIM_MODEL'] = 'none'

        # PX4 SITL needs LD_LIBRARY_PATH for acados shared libs
        acados_lib = os.path.join(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(px4_bin))))))), 
            '..', '..', '..', '..', '..', 'acados-main', 'lib')
        # Fallback: try well-known path
        if not os.path.isdir(acados_lib):
            acados_lib = '/home/yoga/m13/acados-main/lib'
        if os.path.isdir(acados_lib):
            px4_env['LD_LIBRARY_PATH'] = acados_lib + ':' + px4_env.get('LD_LIBRARY_PATH', '')

        # PX4 SITL working directory must be build/px4_sitl_default
        px4_build_dir = os.path.dirname(os.path.dirname(px4_bin))  # .../build/px4_sitl_default
        px4_cwd = px4_build_dir if os.path.isdir(os.path.join(px4_build_dir, 'etc')) else None

        # Clean stale parameters so PX4 re-applies airframe 22003 config
        if px4_cwd:
            for pfile in ('parameters.bson', 'parameters_backup.bson'):
                p = os.path.join(px4_cwd, pfile)
                if os.path.exists(p):
                    os.remove(p)

        # PX4 stdout log file — CRITICAL: do NOT use subprocess.PIPE without
        # reading it.  The 64KB pipe buffer fills up, PX4 blocks on write,
        # and the sim_rcv/sim_send threads deadlock → TCP dies → BrokenPipe.
        px4_log_path = os.path.join(str(_RESULTS_DIR), 'px4_stdout.log')
        os.makedirs(os.path.dirname(px4_log_path), exist_ok=True)
        px4_log_file = open(px4_log_path, 'w', encoding='utf-8')

        px4_proc = subprocess.Popen(
            [px4_bin, '-s', 'etc/init.d-posix/rcS', '-d'],
            env=px4_env,
            cwd=px4_cwd,
            stdout=px4_log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        print(f"  PX4 started (PID={px4_proc.pid}), cwd={px4_cwd}")
        print(f"  PX4 stdout → {px4_log_path}")
        print(f"  Waiting 5s for PX4 TCP init...")
        time.sleep(5)
    else:
        px4_proc = None
        print("  No PX4 binary specified — assuming PX4 is already running")
        print(f"  (If not, start PX4 SITL manually and connect to port {port})")

    try:
        # Run bridge
        from mavlink_bridge import SITLBridge
        bridge = SITLBridge(sitl_cfg, sim_config)
        bridge.port = port
        bridge.run(csv_output=csv_path)
    finally:
        if px4_proc:
            # Check if PX4 is still alive
            retcode = px4_proc.poll()
            if retcode is not None:
                print(f"  PX4 already exited with code {retcode} (signal {-retcode if retcode < 0 else 'N/A'})")
            else:
                print("  Stopping PX4...")
                os.killpg(os.getpgid(px4_proc.pid), signal.SIGTERM)
                px4_proc.wait(timeout=5)
            px4_log_file.close()
            # Print last 80 lines of PX4 output for diagnostics
            print(f"\n  --- PX4 stdout (last 80 lines from {px4_log_path}) ---")
            try:
                with open(px4_log_path, 'r', encoding='utf-8', errors='replace') as f:
                    lines = f.readlines()
                for line in lines[-80:]:
                    print(f"  PX4| {line.rstrip()}")
            except Exception as e:
                print(f"  (could not read PX4 log: {e})")
            print(f"  --- end PX4 stdout ---\n")

    print(f"  Saved: {csv_path}")
    return csv_path


def load_csv(path: str) -> dict:
    """Load flight CSV into numpy arrays.

    Column names are normalized so baseline CSV (from rocket_6dof_sim main
    export, e.g. ``time_s``, ``velocity_x_m_s``, ``altitude_m``, ``quat_w``)
    and SITL bridge CSV (``time``, ``vel_x``, ``altitude``, ``q0``) compare
    against the same canonical keys.
    """
    # Canonical key -> list of possible source column names (first match wins).
    ALIASES = {
        'time':         ['time', 'time_s'],
        'vel_x':        ['vel_x', 'velocity_x_m_s'],
        'vel_y':        ['vel_y', 'velocity_y_m_s'],
        'vel_z':        ['vel_z', 'velocity_z_m_s'],
        'pos_x':        ['pos_x', 'position_x_m'],
        'pos_y':        ['pos_y', 'position_y_m'],
        'pos_z':        ['pos_z', 'position_z_m'],
        'altitude':     ['altitude', 'altitude_m'],
        'altitude_msl': ['altitude_msl', 'alt_msl', 'altitude_lla_m', 'altitude_m'],
        'altitude_agl': ['altitude_agl', 'altitude'],
        'ground_range': ['ground_range', 'ground_range_m'],
        'alpha':        ['alpha', 'alpha_rad'],
        'beta':         ['beta', 'beta_rad'],
        'mach':         ['mach'],
        'q0':           ['q0', 'quat_w'],
        'q1':           ['q1', 'quat_x'],
        'q2':           ['q2', 'quat_y'],
        'q3':           ['q3', 'quat_z'],
        'fin_cmd_1':    ['fin_cmd_1', 'fin_1_rad'],
        'fin_cmd_2':    ['fin_cmd_2', 'fin_2_rad'],
        'fin_cmd_3':    ['fin_cmd_3', 'fin_3_rad'],
        'fin_cmd_4':    ['fin_cmd_4', 'fin_4_rad'],
        'fin_act_1':    ['fin_act_1'],
        'fin_act_2':    ['fin_act_2'],
        'fin_act_3':    ['fin_act_3'],
        'fin_act_4':    ['fin_act_4'],
        'omega_x':      ['omega_x', 'omega_x_rad_s'],
        'omega_y':      ['omega_y', 'omega_y_rad_s'],
        'omega_z':      ['omega_z', 'omega_z_rad_s'],
        'force_x':      ['force_x', 'force_x_N'],
        'force_y':      ['force_y', 'force_y_N'],
        'force_z':      ['force_z', 'force_z_N'],
        'mass':         ['mass', 'mass_kg'],
    }
    raw = {}
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for key in reader.fieldnames:
            raw[key] = []
        for row in reader:
            for key in reader.fieldnames:
                try:
                    raw[key].append(float(row[key]))
                except (ValueError, TypeError):
                    raw[key].append(0.0)
    data = {k: np.array(v) for k, v in raw.items()}

    # Add canonical aliases (non-destructive: keep original names too).
    for canonical, candidates in ALIASES.items():
        if canonical in data:
            continue
        for cand in candidates:
            if cand in data:
                data[canonical] = data[cand]
                break

    # Derive Euler angles from quaternions if not present
    if 'pitch_deg' not in data and all(k in data for k in ['q0', 'q1', 'q2', 'q3']):
        roll, pitch, yaw = _quat_to_euler(data['q0'], data['q1'], data['q2'], data['q3'])
        data['roll_deg'] = roll
        data['pitch_deg'] = pitch
        data['yaw_deg'] = yaw

    # Derive total velocity if not present
    if 'vel_total' not in data and all(k in data for k in ['vel_x', 'vel_y', 'vel_z']):
        data['vel_total'] = np.sqrt(data['vel_x']**2 + data['vel_y']**2 + data['vel_z']**2)
    elif 'velocity_total_m_s' in data:
        data['vel_total'] = data['velocity_total_m_s']

    # Convert alpha/beta to degrees if in radians
    if 'alpha' in data and 'alpha_deg' not in data:
        if len(data['alpha']) > 0 and np.nanmax(np.abs(data['alpha'])) < 2 * np.pi:
            data['alpha_deg'] = np.degrees(data['alpha'])
        else:
            data['alpha_deg'] = data['alpha']
    if 'beta' in data and 'beta_deg' not in data:
        if len(data['beta']) > 0 and np.nanmax(np.abs(data['beta'])) < 2 * np.pi:
            data['beta_deg'] = np.degrees(data['beta'])
        else:
            data['beta_deg'] = data['beta']

    # Derive angular rates in deg/s
    if 'omega_x' in data and 'omega_x_deg_s' not in data:
        if len(data['omega_x']) > 0 and np.nanmax(np.abs(data['omega_x'])) < 200:
            data['omega_x_deg_s'] = np.degrees(data['omega_x'])
            data['omega_y_deg_s'] = np.degrees(data['omega_y'])
            data['omega_z_deg_s'] = np.degrees(data['omega_z'])
        else:
            data['omega_x_deg_s'] = data['omega_x']
            data['omega_y_deg_s'] = data['omega_y']
            data['omega_z_deg_s'] = data['omega_z']

    return data


def compare_and_report(baseline_csv: str, sitl_csv: str,
                       output_html: str = None) -> dict:
    """Compare baseline and SITL results, generate HTML report.
    
    Returns dict with comparison metrics.
    """
    print()
    print("=" * 70)
    print("  STEP 3: Comparing results and generating report")
    print("=" * 70)

    baseline = load_csv(baseline_csv)
    sitl = load_csv(sitl_csv)

    html_path = output_html or str(_RESULTS_DIR / 'sitl_comparison.html')

    # Compute metrics
    metrics = _compute_metrics(baseline, sitl)

    # Print summary
    _print_summary(metrics)

    # Generate HTML
    _generate_html_report(baseline, sitl, metrics, html_path,
                          baseline_csv, sitl_csv)

    print(f"\n  Report saved: {html_path}")
    return metrics


def _compute_metrics(baseline: dict, sitl: dict) -> dict:
    """Compute comparison metrics between baseline and SITL."""
    metrics = {}

    # Time alignment — find common time range
    t_base = baseline.get('time', np.array([]))
    t_sitl = sitl.get('time', np.array([]))

    if len(t_base) == 0 or len(t_sitl) == 0:
        return {'error': 'Empty data'}

    t_max = min(t_base[-1], t_sitl[-1])
    metrics['t_final_baseline'] = float(t_base[-1])
    metrics['t_final_sitl'] = float(t_sitl[-1])
    metrics['t_final_diff'] = abs(t_base[-1] - t_sitl[-1])

    # Interpolate SITL onto baseline timeline for comparison
    common_mask = t_base <= t_max
    t_common = t_base[common_mask]

    def interp_sitl(key):
        if key in sitl and len(sitl[key]) > 0:
            return np.interp(t_common, t_sitl, sitl[key])
        return np.zeros_like(t_common)

    def interp_base(key):
        if key in baseline and len(baseline[key]) > 0:
            return baseline[key][common_mask]
        return np.zeros_like(t_common)

    def pick_altitude_key(ds: dict, other_ds: dict) -> str:
      """Choose altitude key that best matches the other dataset reference.

      Some logs store altitude as MSL while others store AGL. Pick the key
      that minimizes initial-offset mismatch so MAE/CEP are meaningful.
      """
      candidates = [k for k in ('altitude', 'altitude_msl', 'altitude_agl')
              if k in ds and len(ds[k]) > 0]

      if not candidates:
        return 'altitude'

      other_candidates = [k for k in ('altitude', 'altitude_msl', 'altitude_agl')
                if k in other_ds and len(other_ds[k]) > 0]

      if not other_candidates:
        return candidates[0]

      best_key = candidates[0]
      best_score = float('inf')

      for key in candidates:
        a0 = float(ds[key][0])
        score = min(abs(a0 - float(other_ds[ok][0])) for ok in other_candidates)

        if score < best_score:
          best_score = score
          best_key = key

      return best_key

    alt_base_key = pick_altitude_key(baseline, sitl)
    alt_sitl_key = pick_altitude_key(sitl, baseline)

    # Altitude comparison
    alt_base = interp_base(alt_base_key)
    alt_sitl = interp_sitl(alt_sitl_key)
    alt_err = alt_sitl - alt_base
    metrics['altitude_mae'] = float(np.mean(np.abs(alt_err)))
    metrics['altitude_max_err'] = float(np.max(np.abs(alt_err)))
    metrics['altitude_rmse'] = float(np.sqrt(np.mean(alt_err**2)))
    metrics['max_altitude_base'] = float(np.max(alt_base))
    metrics['max_altitude_sitl'] = float(np.max(alt_sitl))
    metrics['_alt_base_key'] = alt_base_key
    metrics['_alt_sitl_key'] = alt_sitl_key

    # Ground range comparison
    rng_base = interp_base('ground_range')
    rng_sitl = interp_sitl('ground_range')
    rng_err = rng_sitl - rng_base
    metrics['range_mae'] = float(np.mean(np.abs(rng_err)))
    metrics['range_max_err'] = float(np.max(np.abs(rng_err)))

    # Impact point (last row)
    for ds, prefix in [(baseline, 'base'), (sitl, 'sitl')]:
      if 'ground_range' in ds and len(ds['ground_range']) > 0:
        metrics[f'impact_range_{prefix}'] = float(ds['ground_range'][-1])

      alt_key = alt_base_key if prefix == 'base' else alt_sitl_key

      if alt_key in ds and len(ds[alt_key]) > 0:
        metrics[f'impact_alt_{prefix}'] = float(ds[alt_key][-1])

    # CEP (Circular Error Probable at impact)
    if all(k in metrics for k in ['impact_range_base', 'impact_range_sitl',
                                   'impact_alt_base', 'impact_alt_sitl']):
        dx = metrics['impact_range_sitl'] - metrics['impact_range_base']
        dz = metrics['impact_alt_sitl'] - metrics['impact_alt_base']
        metrics['impact_cep'] = float(np.sqrt(dx**2 + dz**2))

    # Velocity comparison
    if 'vel_x' in baseline:
        vx_err = interp_sitl('vel_x') - interp_base('vel_x')
        vy_err = interp_sitl('vel_y') - interp_base('vel_y')
        vz_err = interp_sitl('vel_z') - interp_base('vel_z')
        v_err = np.sqrt(vx_err**2 + vy_err**2 + vz_err**2)
        metrics['velocity_mae'] = float(np.mean(v_err))
        metrics['velocity_max_err'] = float(np.max(v_err))

    # Fin deflections
    for ch in range(1, 5):
        key_cmd = f'fin_cmd_{ch}'  # SITL
        key_act = f'control_fins_rad'  # baseline (in comprehensive log)
        # Use available columns
        if key_cmd in sitl:
            pass  # SITL has fin_cmd columns

    # Attitude error
    if all(k in baseline for k in ['q0', 'q1', 'q2', 'q3']):
        q0_err = interp_sitl('q0') - interp_base('q0')
        q1_err = interp_sitl('q1') - interp_base('q1')
        q2_err = interp_sitl('q2') - interp_base('q2')
        q3_err = interp_sitl('q3') - interp_base('q3')
        quat_err = np.sqrt(q0_err**2 + q1_err**2 + q2_err**2 + q3_err**2)
        # Quaternion error → approximate angle (radians)
        angle_err_rad = 2.0 * np.arcsin(np.clip(quat_err / 2.0, 0, 1))
        metrics['attitude_mae_deg'] = float(np.degrees(np.mean(angle_err_rad)))
        metrics['attitude_max_deg'] = float(np.degrees(np.max(angle_err_rad)))

    # Alpha / Mach
    if 'alpha' in baseline:
        alpha_err = np.abs(interp_sitl('alpha') - interp_base('alpha'))
        metrics['alpha_mae_deg'] = float(np.degrees(np.mean(alpha_err)))

    if 'mach' in baseline:
        mach_err = np.abs(interp_sitl('mach') - interp_base('mach'))
        metrics['mach_mae'] = float(np.mean(mach_err))

    # Store time series for plotting
    metrics['_t_common'] = t_common
    metrics['_alt_base'] = alt_base
    metrics['_alt_sitl'] = alt_sitl
    metrics['_rng_base'] = rng_base
    metrics['_rng_sitl'] = rng_sitl

    # Velocity time series
    if 'vel_x' in baseline and 'vel_x' in sitl:
        vb = interp_base('vel_total') if 'vel_total' in baseline else np.sqrt(
            interp_base('vel_x')**2 + interp_base('vel_y')**2 + interp_base('vel_z')**2)
        vs = np.sqrt(interp_sitl('vel_x')**2 + interp_sitl('vel_y')**2 + interp_sitl('vel_z')**2)
        metrics['_vel_base'] = vb
        metrics['_vel_sitl'] = vs

    # Attitude time series (Euler angles)
    if 'pitch_deg' in baseline:
        metrics['_pitch_base'] = interp_base('pitch_deg')
    if 'pitch_deg' in sitl:
        metrics['_pitch_sitl'] = np.interp(t_common, t_sitl, sitl['pitch_deg'])
    if 'yaw_deg' in baseline:
        metrics['_yaw_base'] = interp_base('yaw_deg')
    if 'yaw_deg' in sitl:
        metrics['_yaw_sitl'] = np.interp(t_common, t_sitl, sitl['yaw_deg'])
    if 'roll_deg' in baseline:
        metrics['_roll_base'] = interp_base('roll_deg')
    if 'roll_deg' in sitl:
        metrics['_roll_sitl'] = np.interp(t_common, t_sitl, sitl['roll_deg'])

    # Attitude error (degrees)
    if all(k in baseline for k in ['q0', 'q1', 'q2', 'q3']):
        angle_err_deg = np.degrees(angle_err_rad)
        metrics['_att_err_deg'] = angle_err_deg

    # Alpha / Beta / Mach time series
    if 'alpha_deg' in baseline and 'alpha_deg' in sitl:
        metrics['_alpha_base'] = interp_base('alpha_deg')
        metrics['_alpha_sitl'] = np.interp(t_common, t_sitl, sitl['alpha_deg'])
    if 'beta_deg' in baseline and 'beta_deg' in sitl:
        metrics['_beta_base'] = interp_base('beta_deg')
        metrics['_beta_sitl'] = np.interp(t_common, t_sitl, sitl['beta_deg'])
    if 'mach' in baseline and 'mach' in sitl:
        metrics['_mach_base'] = interp_base('mach')
        metrics['_mach_sitl'] = interp_sitl('mach')

    # Fin deflection time series (in degrees)
    for ch in range(1, 5):
        key = f'fin_cmd_{ch}'
        if key in sitl and len(sitl[key]) > 0:
            metrics[f'_fin_sitl_{ch}'] = np.degrees(np.interp(t_common, t_sitl, sitl[key]))
        if key in baseline and len(baseline[key]) > 0:
            metrics[f'_fin_base_{ch}'] = np.degrees(interp_base(key))

    # Angular rates time series
    if 'omega_y_deg_s' in baseline and 'omega_y_deg_s' in sitl:
        metrics['_omegay_base'] = interp_base('omega_y_deg_s')
        metrics['_omegay_sitl'] = np.interp(t_common, t_sitl, sitl['omega_y_deg_s'])
    if 'omega_z_deg_s' in baseline and 'omega_z_deg_s' in sitl:
        metrics['_omegaz_base'] = interp_base('omega_z_deg_s')
        metrics['_omegaz_sitl'] = np.interp(t_common, t_sitl, sitl['omega_z_deg_s'])

    # Flight phase from baseline (for timeline bands)
    if 'flight_phase' in baseline:
        metrics['_flight_phase'] = baseline.get('flight_phase', np.array([]))
        metrics['_t_base'] = t_base

    # Pass/Fail criteria
    metrics['pass_impact_cep'] = metrics.get('impact_cep', 999) < 50.0  # < 50m
    metrics['pass_altitude'] = metrics.get('altitude_mae', 999) < 20.0  # < 20m MAE
    metrics['pass_velocity'] = metrics.get('velocity_mae', 999) < 5.0   # < 5 m/s MAE
    metrics['pass_attitude'] = metrics.get('attitude_max_deg', 999) < 5.0  # < 5°
    metrics['overall_pass'] = all([
        metrics.get('pass_impact_cep', False),
        metrics.get('pass_altitude', False),
        metrics.get('pass_velocity', False),
        metrics.get('pass_attitude', False),
    ])

    return metrics


def _print_summary(metrics: dict):
    """Print comparison summary to console."""
    def _status(key):
        return "✅ PASS" if metrics.get(key, False) else "❌ FAIL"

    print()
    print("  ┌─────────────────────────────────────────────────────┐")
    print("  │            SITL vs Baseline Comparison              │")
    print("  ├──────────────────────┬──────────────┬───────────────┤")
    print(f"  │ Flight time (base)   │ {metrics.get('t_final_baseline', 0):>10.3f}s │               │")
    print(f"  │ Flight time (SITL)   │ {metrics.get('t_final_sitl', 0):>10.3f}s │   Δ={metrics.get('t_final_diff', 0):.3f}s   │")
    print("  ├──────────────────────┼──────────────┼───────────────┤")
    print(f"  │ Impact CEP           │ {metrics.get('impact_cep', 0):>10.2f}m │ {_status('pass_impact_cep'):>13s} │")
    print(f"  │ Altitude MAE         │ {metrics.get('altitude_mae', 0):>10.2f}m │ {_status('pass_altitude'):>13s} │")
    print(f"  │ Velocity MAE         │ {metrics.get('velocity_mae', 0):>8.2f}m/s │ {_status('pass_velocity'):>13s} │")
    print(f"  │ Attitude max err     │ {metrics.get('attitude_max_deg', 0):>9.2f}° │ {_status('pass_attitude'):>13s} │")
    print("  ├──────────────────────┴──────────────┴───────────────┤")

    overall = metrics.get('overall_pass', False)
    tag = "✅ ALL TESTS PASSED" if overall else "❌ SOME TESTS FAILED"
    print(f"  │  {tag:^49s}  │")
    print("  └─────────────────────────────────────────────────────┘")


# ─── Phase colors (matching advanced_analysis.py) ────────────────────────────

PHASE_COLORS = {
    "LAUNCH": "#ff6b6b", "BOOST": "#ff6b6b", "PITCH_PROGRAM": "#ffa94d",
    "SUSTAIN": "#ffd43b", "CRUISE": "#69db7c", "BALLISTIC": "#4dabf7",
    "TERMINAL": "#da77f2", "COAST": "#868e96",
}


def _plotly_div(fig, div_id=None):
    """Render a Plotly figure to an HTML div string (no full-page wrapper)."""
    config = {"responsive": True, "displayModeBar": True,
              "modeBarButtonsToRemove": ["lasso2d", "select2d"]}
    html = pio.to_html(fig, full_html=False, include_plotlyjs=False,
                       config=config, div_id=div_id)
    return f'<div class="chart-container">{html}</div>'


def _add_phase_bands(fig, metrics, row=None, col=None):
    """Add semi-transparent flight phase background bands to a Plotly figure."""
    fp = metrics.get('_flight_phase')
    t_base = metrics.get('_t_base')
    if fp is None or t_base is None or len(fp) == 0:
        return
    # fp may be a numpy array of strings or floats (0.0 if non-string)
    try:
        phases_arr = np.array(fp, dtype=str)
    except Exception:
        return
    if phases_arr[0] == '0.0':
        return  # not real phase data
    from collections import OrderedDict
    seen = OrderedDict()
    for i, ph in enumerate(phases_arr):
        if ph not in seen:
            seen[ph] = [i, i]
        else:
            seen[ph][1] = i
    for ph, (i0, i1) in seen.items():
        color = PHASE_COLORS.get(ph, '#333')
        kwargs = {}
        if row is not None:
            kwargs['row'] = row
        if col is not None:
            kwargs['col'] = col
        fig.add_vrect(
            x0=float(t_base[i0]), x1=float(t_base[i1]),
            fillcolor=color, opacity=0.07, layer="below", line_width=0,
            **kwargs
        )


def _generate_html_report(baseline: dict, sitl: dict, metrics: dict,
                          output_path: str, baseline_csv: str,
                          sitl_csv: str):
    """Generate professional interactive HTML comparison report using Plotly."""

    if not HAS_PLOTLY:
        print("  WARNING: plotly not installed. Install with: pip install plotly")
        print("  Falling back to minimal text report.")
        with open(output_path, 'w') as f:
            f.write("<html><body><h1>Plotly not installed</h1></body></html>")
        return

    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    t = metrics.get('_t_common', np.array([]))
    alt_base = metrics.get('_alt_base', np.array([]))
    alt_sitl = metrics.get('_alt_sitl', np.array([]))
    rng_base = metrics.get('_rng_base', np.array([]))
    rng_sitl = metrics.get('_rng_sitl', np.array([]))

    LINE_BASE = dict(color='#3b82f6', width=2)
    LINE_SITL = dict(color='#f59e0b', width=2, dash='dash')
    LINE_ERR  = dict(color='#ef4444', width=1.5)

    # ── Fig 1: Trajectory Overview (2×2) ────────────────────────────────────
    fig_traj = make_subplots(
        rows=2, cols=2,
        subplot_titles=("Altitude vs Time", "Ground Range vs Time",
                        "Trajectory (Range vs Altitude)", "Speed vs Time"),
        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig_traj.add_trace(go.Scatter(x=t, y=alt_base, name='Baseline', line=LINE_BASE), row=1, col=1)
    fig_traj.add_trace(go.Scatter(x=t, y=alt_sitl, name='SITL', line=LINE_SITL), row=1, col=1)
    fig_traj.add_trace(go.Scatter(x=t, y=rng_base, name='Baseline', line=LINE_BASE, showlegend=False), row=1, col=2)
    fig_traj.add_trace(go.Scatter(x=t, y=rng_sitl, name='SITL', line=LINE_SITL, showlegend=False), row=1, col=2)
    fig_traj.add_trace(go.Scatter(x=rng_base, y=alt_base, name='Baseline', line=LINE_BASE, showlegend=False), row=2, col=1)
    fig_traj.add_trace(go.Scatter(x=rng_sitl, y=alt_sitl, name='SITL', line=LINE_SITL, showlegend=False), row=2, col=1)
    if '_vel_base' in metrics:
        fig_traj.add_trace(go.Scatter(x=t, y=metrics['_vel_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=2, col=2)
        fig_traj.add_trace(go.Scatter(x=t, y=metrics['_vel_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=2, col=2)
    fig_traj.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig_traj.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig_traj.update_xaxes(title_text="Range (m)", row=2, col=1)
    fig_traj.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig_traj.update_yaxes(title_text="Alt (m)", row=1, col=1)
    fig_traj.update_yaxes(title_text="Range (m)", row=1, col=2)
    fig_traj.update_yaxes(title_text="Alt (m)", row=2, col=1)
    fig_traj.update_yaxes(title_text="Speed (m/s)", row=2, col=2)
    _add_phase_bands(fig_traj, metrics, row=1, col=1)
    _add_phase_bands(fig_traj, metrics, row=1, col=2)
    _add_phase_bands(fig_traj, metrics, row=2, col=2)
    fig_traj.update_layout(height=700, template='plotly_white',
                           legend=dict(orientation='h', yanchor='bottom', y=1.02))

    # ── Fig 2: Error Time Series (2×2) ──────────────────────────────────────
    fig_err = make_subplots(
        rows=2, cols=2,
        subplot_titles=("Altitude Error (SITL − Baseline)", "Range Error",
                        "Velocity Error (magnitude)", "Attitude Error"),
        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig_err.add_trace(go.Scatter(x=t, y=alt_sitl - alt_base, name='ΔAlt',
                                 line=LINE_ERR, fill='tozeroy',
                                 fillcolor='rgba(239,68,68,0.08)'), row=1, col=1)
    fig_err.add_trace(go.Scatter(x=t, y=rng_sitl - rng_base, name='ΔRange',
                                 line=dict(color='#f97316', width=1.5), fill='tozeroy',
                                 fillcolor='rgba(249,115,22,0.08)', showlegend=False), row=1, col=2)
    if '_vel_base' in metrics:
        vel_err = metrics['_vel_sitl'] - metrics['_vel_base']
        fig_err.add_trace(go.Scatter(x=t, y=vel_err, name='ΔSpeed',
                                     line=dict(color='#8b5cf6', width=1.5), fill='tozeroy',
                                     fillcolor='rgba(139,92,246,0.08)', showlegend=False), row=2, col=1)
    if '_att_err_deg' in metrics:
        fig_err.add_trace(go.Scatter(x=t, y=metrics['_att_err_deg'], name='Att Error',
                                     line=dict(color='#ec4899', width=1.5), fill='tozeroy',
                                     fillcolor='rgba(236,72,153,0.08)', showlegend=False), row=2, col=2)
    fig_err.update_xaxes(title_text="Time (s)")
    fig_err.update_yaxes(title_text="ΔAlt (m)", row=1, col=1)
    fig_err.update_yaxes(title_text="ΔRange (m)", row=1, col=2)
    fig_err.update_yaxes(title_text="ΔSpeed (m/s)", row=2, col=1)
    fig_err.update_yaxes(title_text="Error (deg)", row=2, col=2)
    for r in [1, 2]:
        for c in [1, 2]:
            fig_err.add_hline(y=0, line_dash='dot', line_color='gray', opacity=0.4, row=r, col=c)
            _add_phase_bands(fig_err, metrics, row=r, col=c)
    fig_err.update_layout(height=700, template='plotly_white')

    # ── Fig 3: Attitude Comparison (2×2) ────────────────────────────────────
    fig_att = make_subplots(
        rows=2, cols=2,
        subplot_titles=("Pitch Angle", "Yaw Angle",
                        "Roll Angle", "Pitch Rate (q)"),
        vertical_spacing=0.12, horizontal_spacing=0.08)
    has_att = False
    if '_pitch_base' in metrics:
        has_att = True
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_pitch_base'], name='Baseline', line=LINE_BASE), row=1, col=1)
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_pitch_sitl'], name='SITL', line=LINE_SITL), row=1, col=1)
    if '_yaw_base' in metrics:
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_yaw_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=1, col=2)
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_yaw_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=1, col=2)
    if '_roll_base' in metrics:
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_roll_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=2, col=1)
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_roll_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=2, col=1)
    if '_omegay_base' in metrics:
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_omegay_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=2, col=2)
        fig_att.add_trace(go.Scatter(x=t, y=metrics['_omegay_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=2, col=2)
    fig_att.update_xaxes(title_text="Time (s)")
    fig_att.update_yaxes(title_text="Pitch (deg)", row=1, col=1)
    fig_att.update_yaxes(title_text="Yaw (deg)", row=1, col=2)
    # Match roll Y-axis to pitch scale so tiny values aren't visually exaggerated
    pitch_range = None
    if '_pitch_base' in metrics:
        pmax = max(abs(metrics['_pitch_base'].min()), abs(metrics['_pitch_base'].max()),
                   abs(metrics['_pitch_sitl'].min()), abs(metrics['_pitch_sitl'].max()))
        pitch_range = [-pmax * 1.1, pmax * 1.1]
    fig_att.update_yaxes(title_text="Roll (deg)", row=2, col=1,
                         range=pitch_range if pitch_range else [-1, 1])
    fig_att.update_yaxes(title_text="q (deg/s)", row=2, col=2)
    for r in [1, 2]:
        for c in [1, 2]:
            _add_phase_bands(fig_att, metrics, row=r, col=c)
    fig_att.update_layout(height=700, template='plotly_white',
                          legend=dict(orientation='h', yanchor='bottom', y=1.02))

    # ── Fig 4: Aero Comparison (1×3) ────────────────────────────────────────
    fig_aero = make_subplots(
        rows=1, cols=3,
        subplot_titles=("Angle of Attack (α)", "Sideslip (β)", "Mach Number"),
        horizontal_spacing=0.08)
    has_aero = False
    if '_alpha_base' in metrics:
        has_aero = True
        fig_aero.add_trace(go.Scatter(x=t, y=metrics['_alpha_base'], name='Baseline', line=LINE_BASE), row=1, col=1)
        fig_aero.add_trace(go.Scatter(x=t, y=metrics['_alpha_sitl'], name='SITL', line=LINE_SITL), row=1, col=1)
    if '_beta_base' in metrics:
        fig_aero.add_trace(go.Scatter(x=t, y=metrics['_beta_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=1, col=2)
        fig_aero.add_trace(go.Scatter(x=t, y=metrics['_beta_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=1, col=2)
    if '_mach_base' in metrics:
        fig_aero.add_trace(go.Scatter(x=t, y=metrics['_mach_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=1, col=3)
        fig_aero.add_trace(go.Scatter(x=t, y=metrics['_mach_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=1, col=3)
    fig_aero.update_xaxes(title_text="Time (s)")
    fig_aero.update_yaxes(title_text="α (deg)", row=1, col=1)
    fig_aero.update_yaxes(title_text="β (deg)", row=1, col=2)
    fig_aero.update_yaxes(title_text="Mach", row=1, col=3)
    for c in [1, 2, 3]:
        _add_phase_bands(fig_aero, metrics, row=1, col=c)
    fig_aero.update_layout(height=400, template='plotly_white',
                           legend=dict(orientation='h', yanchor='bottom', y=1.05))

    # ── Fig 5: Fin Deflections (2×2) ────────────────────────────────────────
    fin_colors = ['#22c55e', '#a855f7', '#ec4899', '#14b8a6']
    fig_fins = make_subplots(
        rows=2, cols=2,
        subplot_titles=("Fin 1 (Baseline vs SITL)", "Fin 2",
                        "Fin 3", "Fin 4"),
        vertical_spacing=0.12, horizontal_spacing=0.08)
    has_fins = False
    for ch in range(1, 5):
        r = (ch - 1) // 2 + 1
        c = (ch - 1) % 2 + 1
        clr = fin_colors[ch - 1]
        base_key = f'_fin_base_{ch}'
        sitl_key = f'_fin_sitl_{ch}'
        if base_key in metrics:
            has_fins = True
            fig_fins.add_trace(go.Scatter(
                x=t, y=metrics[base_key], name=f'Fin{ch} Base' if ch == 1 else None,
                line=dict(color=clr, width=2), legendgroup='base',
                showlegend=(ch == 1)), row=r, col=c)
        if sitl_key in metrics:
            has_fins = True
            fig_fins.add_trace(go.Scatter(
                x=t, y=metrics[sitl_key], name=f'Fin{ch} SITL' if ch == 1 else None,
                line=dict(color=clr, width=1.5, dash='dash'), legendgroup='sitl',
                showlegend=(ch == 1)), row=r, col=c)
        fig_fins.update_yaxes(title_text="δ (deg)", row=r, col=c)
    fig_fins.update_xaxes(title_text="Time (s)")
    fig_fins.update_layout(height=700, template='plotly_white',
                           legend=dict(orientation='h', yanchor='bottom', y=1.02))

    # ── Fig 6: Angular Rates (1×2) ──────────────────────────────────────────
    fig_rates = make_subplots(
        rows=1, cols=2,
        subplot_titles=("Pitch Rate q (deg/s)", "Yaw Rate r (deg/s)"),
        horizontal_spacing=0.08)
    has_rates = False
    if '_omegay_base' in metrics:
        has_rates = True
        fig_rates.add_trace(go.Scatter(x=t, y=metrics['_omegay_base'], name='Baseline', line=LINE_BASE), row=1, col=1)
        fig_rates.add_trace(go.Scatter(x=t, y=metrics['_omegay_sitl'], name='SITL', line=LINE_SITL), row=1, col=1)
    if '_omegaz_base' in metrics:
        fig_rates.add_trace(go.Scatter(x=t, y=metrics['_omegaz_base'], name='Baseline', line=LINE_BASE, showlegend=False), row=1, col=2)
        fig_rates.add_trace(go.Scatter(x=t, y=metrics['_omegaz_sitl'], name='SITL', line=LINE_SITL, showlegend=False), row=1, col=2)
    fig_rates.update_xaxes(title_text="Time (s)")
    fig_rates.update_yaxes(title_text="q (deg/s)", row=1, col=1)
    fig_rates.update_yaxes(title_text="r (deg/s)", row=1, col=2)
    fig_rates.update_layout(height=400, template='plotly_white',
                            legend=dict(orientation='h', yanchor='bottom', y=1.05))

    # ── Build HTML ──────────────────────────────────────────────────────────
    overall = metrics.get('overall_pass', False)
    pass_color = '#22c55e' if overall else '#ef4444'
    pass_text = 'ALL TESTS PASSED — SITL يطابق المرجع' if overall else 'SOME TESTS FAILED — يوجد اختلاف'

    def _badge(verdict, text=None):
        t = text or verdict
        cls = 'pass' if verdict else 'fail'
        return f'<span class="badge {cls}">{html_escape(str(t))}</span>'

    def _metric_card(label, value, sub='', color='var(--accent)'):
        return (f'<div class="metric-box">'
                f'<div class="value" style="color:{color}">{value}</div>'
                f'<div class="label">{html_escape(label)}</div>'
                f'<div class="sub">{html_escape(sub)}</div></div>')

    # Summary stat cards
    cep = metrics.get('impact_cep', 0)
    cep_color = '#22c55e' if cep < 10 else ('#f59e0b' if cep < 50 else '#ef4444')
    alt_mae = metrics.get('altitude_mae', 0)
    alt_color = '#22c55e' if alt_mae < 5 else ('#f59e0b' if alt_mae < 20 else '#ef4444')
    vel_mae = metrics.get('velocity_mae', 0)
    vel_color = '#22c55e' if vel_mae < 1 else ('#f59e0b' if vel_mae < 5 else '#ef4444')
    att_max = metrics.get('attitude_max_deg', 0)
    att_color = '#22c55e' if att_max < 2 else ('#f59e0b' if att_max < 5 else '#ef4444')

    cards_html = f"""<div class="grid grid-4">
      {_metric_card('Impact CEP', f'{cep:.2f} m', 'Circular Error Probable', cep_color)}
      {_metric_card('Altitude MAE', f'{alt_mae:.2f} m', f'RMSE: {metrics.get("altitude_rmse", 0):.2f} m', alt_color)}
      {_metric_card('Velocity MAE', f'{vel_mae:.2f} m/s', f'Max: {metrics.get("velocity_max_err", 0):.2f} m/s', vel_color)}
      {_metric_card('Attitude Max', f'{att_max:.2f}°', f'MAE: {metrics.get("attitude_mae_deg", 0):.2f}°', att_color)}
    </div>
    <div class="grid grid-4" style="margin-top:8px">
      {_metric_card('Flight Time (Base)', f'{metrics.get("t_final_baseline", 0):.3f} s', '')}
      {_metric_card('Flight Time (SITL)', f'{metrics.get("t_final_sitl", 0):.3f} s', f'Δ = {metrics.get("t_final_diff", 0):.3f} s')}
      {_metric_card('Max Alt (Base)', f'{metrics.get("max_altitude_base", 0):.0f} m', '')}
      {_metric_card('Max Alt (SITL)', f'{metrics.get("max_altitude_sitl", 0):.0f} m', '')}
    </div>
    <div class="grid grid-4" style="margin-top:8px">
      {_metric_card('Impact Range (Base)', f'{metrics.get("impact_range_base", 0):.0f} m', '')}
      {_metric_card('Impact Range (SITL)', f'{metrics.get("impact_range_sitl", 0):.0f} m', '')}
      {_metric_card('Alpha MAE', f'{metrics.get("alpha_mae_deg", 0):.3f}°', '')}
      {_metric_card('Mach MAE', f'{metrics.get("mach_mae", 0):.4f}', '')}
    </div>"""

    # Metrics table
    test_items = [
        ('Impact CEP', 'impact_cep', 'm', '< 50 m', 'pass_impact_cep'),
        ('Altitude MAE', 'altitude_mae', 'm', '< 20 m', 'pass_altitude'),
        ('Altitude RMSE', 'altitude_rmse', 'm', '—', None),
        ('Altitude Max Error', 'altitude_max_err', 'm', '—', None),
        ('Velocity MAE', 'velocity_mae', 'm/s', '< 5 m/s', 'pass_velocity'),
        ('Velocity Max Error', 'velocity_max_err', 'm/s', '—', None),
        ('Attitude Max Error', 'attitude_max_deg', '°', '< 5°', 'pass_attitude'),
        ('Attitude MAE', 'attitude_mae_deg', '°', '—', None),
        ('Alpha MAE', 'alpha_mae_deg', '°', '—', None),
        ('Mach MAE', 'mach_mae', '', '—', None),
        ('Range MAE', 'range_mae', 'm', '—', None),
        ('Range Max Error', 'range_max_err', 'm', '—', None),
        ('Flight Time Diff', 't_final_diff', 's', '—', None),
    ]
    metric_rows = ''
    for label, key, unit, criteria, pass_key in test_items:
        val = metrics.get(key)
        if val is None:
            continue
        val_str = f'{val:.4f}' if abs(val) < 1 else f'{val:.2f}'
        if pass_key:
            passed = metrics.get(pass_key, False)
            badge_html = _badge(passed, '✅ PASS' if passed else '❌ FAIL')
        else:
            badge_html = ''
        metric_rows += f'<tr><td>{html_escape(label)}</td><td class="mono">{val_str} {unit}</td><td>{criteria}</td><td>{badge_html}</td></tr>'

    # Build tabs
    tab_traj_html = _plotly_div(fig_traj)
    tab_err_html = _plotly_div(fig_err)
    tab_att_html = _plotly_div(fig_att) if has_att else '<p class="muted">Attitude data not available</p>'
    tab_aero_html = _plotly_div(fig_aero) if has_aero else '<p class="muted">Aero data not available</p>'
    tab_fins_html = _plotly_div(fig_fins) if has_fins else '<p class="muted">Fin data not available</p>'
    tab_rates_html = _plotly_div(fig_rates) if has_rates else '<p class="muted">Angular rate data not available</p>'

    # Phase timeline bar
    phase_bar = ''
    fp = metrics.get('_flight_phase')
    t_base = metrics.get('_t_base')
    if fp is not None and t_base is not None:
        try:
            phases_arr = np.array(fp, dtype=str)
            if phases_arr[0] != '0.0':
                from collections import OrderedDict
                seen = OrderedDict()
                for i, ph in enumerate(phases_arr):
                    if ph not in seen:
                        seen[ph] = [float(t_base[i]), float(t_base[i])]
                    else:
                        seen[ph][1] = float(t_base[i])
                total_t = float(t_base[-1] - t_base[0])
                phase_bar = '<div class="phase-bar">'
                for ph, (t0, t1) in seen.items():
                    dur = t1 - t0
                    pct = max(2, dur / total_t * 100) if total_t > 0 else 10
                    c = PHASE_COLORS.get(ph, '#333')
                    phase_bar += (f'<div class="phase-seg" style="width:{pct:.1f}%;background:{c}"'
                                  f' title="{ph}: {dur:.2f}s">{ph}<br>{dur:.1f}s</div>')
                phase_bar += '</div>'
        except Exception:
            pass

    # CSS
    css = """
:root{--pass:#22c55e;--warn:#f59e0b;--fail:#ef4444;--bg:#0f172a;--card:#1e293b;
--border:#334155;--text:#e2e8f0;--text-secondary:#94a3b8;--text-muted:#64748b;
--accent:#3b82f6;--hover:#334155;--th-bg:#334155}
[data-theme="light"]{--bg:#fafafa;--card:#fff;--border:#e0e0e0;--text:#212121;
--text-secondary:#666;--text-muted:#999;--accent:#1976d2;--hover:#f0f7ff;--th-bg:#f5f5f5}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);
line-height:1.6;padding:0;transition:background .3s,color .3s}
.container{max-width:1440px;margin:0 auto;padding:20px}
header{text-align:center;padding:30px 20px;background:linear-gradient(135deg,#1e293b 0%,#0f172a 100%);
border-bottom:3px solid var(--accent)}
header h1{font-size:2em;color:#f8fafc;margin-bottom:5px}
header .subtitle{color:#94a3b8;font-size:1.1em}
.overall-badge{display:inline-block;margin-top:15px;padding:12px 35px;border-radius:8px;
font-size:1.3em;font-weight:bold;color:white}
.card{background:var(--card);border-radius:10px;padding:18px;margin:16px 0;
border:1px solid var(--border);box-shadow:0 1px 4px rgba(0,0,0,.12);transition:background .3s}
.card h2{color:var(--accent);font-size:1.2em;margin-bottom:12px;padding-bottom:8px;
border-bottom:1px solid var(--border);border-left:4px solid var(--accent);padding-left:10px}
table{width:100%;border-collapse:collapse;font-size:.9em}
th{background:var(--th-bg);color:var(--text);padding:10px 12px;text-align:left;
border-bottom:2px solid var(--border);font-weight:600}
td{padding:8px 12px;border-bottom:1px solid var(--border)}
tr:hover{background:var(--hover)}
.mono{font-family:'Consolas','Fira Code',monospace}
.badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.78rem;font-weight:700;color:#fff}
.badge.pass{background:var(--pass)}.badge.fail{background:var(--fail)}.badge.warn{background:var(--warn)}
.grid{display:grid;gap:14px}.grid-2{display:grid;grid-template-columns:1fr 1fr;gap:16px}
.grid-3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}
.grid-4{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}
.metric-box{text-align:center;padding:14px;background:var(--bg);border-radius:8px;border:1px solid var(--border)}
.metric-box .value{font-size:1.7rem;font-weight:700;color:var(--accent)}
.metric-box .label{font-size:.75rem;color:var(--text-secondary);text-transform:uppercase;letter-spacing:.5px}
.metric-box .sub{font-size:.7rem;color:var(--text-muted)}
.chart-container{margin:12px 0}
.tabs{display:flex;gap:4px;border-bottom:2px solid var(--border);flex-wrap:wrap;margin-top:20px}
.tab-btn{padding:10px 22px;border:none;background:none;cursor:pointer;font-size:.9rem;
font-weight:600;border-bottom:3px solid transparent;color:var(--text-secondary);transition:.2s}
.tab-btn:hover{color:var(--accent)}.tab-btn.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-panel{display:none;padding:18px 0}.tab-panel.active{display:block}
.phase-bar{display:flex;height:32px;border-radius:6px;overflow:hidden;margin:10px 0}
.phase-seg{display:flex;align-items:center;justify-content:center;color:#fff;
font-size:.68rem;font-weight:700;min-width:30px}
.muted{color:var(--text-muted);font-size:.9em;padding:30px;text-align:center}
.toolbar{display:flex;align-items:center;gap:12px;margin-bottom:16px}
.theme-toggle{background:var(--card);border:1px solid var(--border);border-radius:20px;
padding:6px 14px;cursor:pointer;font-size:.85rem;color:var(--text);transition:.2s;
display:flex;align-items:center;gap:6px}
.theme-toggle:hover{border-color:var(--accent)}
.meta{color:var(--text-muted);font-size:.82em;text-align:center;margin-top:30px;padding:15px}
@media(max-width:900px){.grid-2,.grid-3,.grid-4{grid-template-columns:1fr}}
"""

    # JavaScript
    js = """
function openTab(evt,tabId){
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  evt.currentTarget.classList.add('active');
  document.getElementById(tabId).querySelectorAll('.js-plotly-plot').forEach(function(p){Plotly.Plots.resize(p);});
}
(function(){
  var saved=localStorage.getItem('sitl_theme')||'dark';
  if(saved==='light') document.documentElement.setAttribute('data-theme','light');
  window.addEventListener('DOMContentLoaded',function(){
    var btn=document.getElementById('theme-toggle');
    if(!btn)return;
    _updateLabel(btn,saved);
    btn.addEventListener('click',function(){
      var cur=document.documentElement.getAttribute('data-theme')||'';
      var next=cur==='light'?'':'light';
      if(next) document.documentElement.setAttribute('data-theme','light');
      else document.documentElement.removeAttribute('data-theme');
      var theme=next?'light':'dark';
      localStorage.setItem('sitl_theme',theme);
      _updateLabel(btn,theme);
      var bg=theme==='dark'?'#1e293b':'#fff';
      var fg=theme==='dark'?'#e0e0e0':'#444';
      document.querySelectorAll('.js-plotly-plot').forEach(function(p){
        Plotly.relayout(p,{'paper_bgcolor':bg,'plot_bgcolor':bg,'font.color':fg});
      });
    });
  });
  function _updateLabel(btn,t){btn.textContent=t==='dark'?'\\u2600 Light Mode':'\\u263e Dark Mode';}
})();
"""

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SITL Comparison — M130 Rocket</title>
<script src="https://cdn.plot.ly/plotly-2.35.0.min.js"></script>
<style>{css}</style>
</head>
<body>

<header>
  <h1>M130 SITL Comparison Report</h1>
  <p class="subtitle">Python MPC (Baseline) vs C++ rocket_mpc (PX4 SITL)</p>
  <div class="overall-badge" style="background:{pass_color}">{pass_text}</div>
</header>

<div class="container">

<div class="toolbar">
  <button id="theme-toggle" class="theme-toggle">☀ Light Mode</button>
</div>

{cards_html}

{phase_bar}

<div class="tabs">
  <button class="tab-btn active" onclick="openTab(event,'tab-overview')">Overview</button>
  <button class="tab-btn" onclick="openTab(event,'tab-traj')">Trajectory</button>
  <button class="tab-btn" onclick="openTab(event,'tab-errors')">Errors</button>
  <button class="tab-btn" onclick="openTab(event,'tab-attitude')">Attitude</button>
  <button class="tab-btn" onclick="openTab(event,'tab-aero')">Aerodynamics</button>
  <button class="tab-btn" onclick="openTab(event,'tab-fins')">Fins</button>
  <button class="tab-btn" onclick="openTab(event,'tab-rates')">Angular Rates</button>
</div>

<!-- Overview Tab -->
<div id="tab-overview" class="tab-panel active">
  <div class="card">
    <h2>Detailed Metrics — مقاييس تفصيلية</h2>
    <table>
      <thead><tr><th>Metric</th><th>Value</th><th>Criteria</th><th>Status</th></tr></thead>
      <tbody>{metric_rows}</tbody>
    </table>
  </div>
</div>

<!-- Trajectory Tab -->
<div id="tab-traj" class="tab-panel">
  <div class="card">
    <h2>Trajectory Comparison</h2>
    <p style="color:var(--text-secondary);font-size:.85rem;margin-bottom:12px">
      Blue = Python baseline (internal MPC) &nbsp;|&nbsp; Orange dashed = PX4 SITL (C++ rocket_mpc).
      Interactive: zoom, pan, hover for values, double-click to reset.
    </p>
    {tab_traj_html}
  </div>
</div>

<!-- Errors Tab -->
<div id="tab-errors" class="tab-panel">
  <div class="card">
    <h2>Error Time Series (SITL − Baseline)</h2>
    <p style="color:var(--text-secondary);font-size:.85rem;margin-bottom:12px">
      Deviation of SITL from baseline over flight time. Shaded area shows error magnitude.
      Background bands indicate flight phases from baseline simulation.
    </p>
    {tab_err_html}
  </div>
</div>

<!-- Attitude Tab -->
<div id="tab-attitude" class="tab-panel">
  <div class="card">
    <h2>Attitude Comparison — مقارنة التوجيه</h2>
    <p style="color:var(--text-secondary);font-size:.85rem;margin-bottom:12px">
      Euler angles derived from quaternions. Baseline (solid) vs SITL (dashed).
    </p>
    {tab_att_html}
  </div>
</div>

<!-- Aero Tab -->
<div id="tab-aero" class="tab-panel">
  <div class="card">
    <h2>Aerodynamic Parameters — المعاملات الايروديناميكية</h2>
    <p style="color:var(--text-secondary);font-size:.85rem;margin-bottom:12px">
      Angle of attack (α), sideslip (β), and Mach number comparison.
    </p>
    {tab_aero_html}
  </div>
</div>

<!-- Fins Tab -->
<div id="tab-fins" class="tab-panel">
  <div class="card">
    <h2>Fin Deflections — انحرافات الزعانف</h2>
    <p style="color:var(--text-secondary);font-size:.85rem;margin-bottom:12px">
      Fin command deflections: Baseline (solid) vs SITL (dashed). All values in degrees.
    </p>
    {tab_fins_html}
  </div>
</div>

<!-- Angular Rates Tab -->
<div id="tab-rates" class="tab-panel">
  <div class="card">
    <h2>Angular Rates — معدلات الدوران</h2>
    {tab_rates_html}
  </div>
</div>

<p class="meta">
  Generated: {timestamp}<br>
  Baseline: {html_escape(os.path.basename(baseline_csv))} &nbsp;|&nbsp;
  SITL: {html_escape(os.path.basename(sitl_csv))}
</p>

</div>

<script>{js}</script>
</body>
</html>"""

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)


# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='M130 SITL Test Runner — baseline + SITL + comparison')
    parser.add_argument('--sim-config', type=str, default=None,
                        help='Simulation config YAML')
    parser.add_argument('--sitl-config', type=str, default=None,
                        help='SITL bridge config YAML')
    parser.add_argument('--px4-bin', type=str, default=None,
                        help='Path to PX4 SITL binary')
    parser.add_argument('--port', type=int, default=4560,
                        help='MAVLink bridge port')
    parser.add_argument('--baseline-only', action='store_true',
                        help='Run only Python baseline')
    parser.add_argument('--sitl-only', action='store_true',
                        help='Run only SITL bridge (skip baseline)')
    parser.add_argument('--compare-only', action='store_true',
                        help='Only generate comparison from existing CSVs')
    parser.add_argument('--baseline-csv', type=str, default=None,
                        help='Path to existing baseline CSV')
    parser.add_argument('--sitl-csv', type=str, default=None,
                        help='Path to existing SITL CSV')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for results')
    args = parser.parse_args()

    results_dir = args.output_dir or str(_RESULTS_DIR)
    os.makedirs(results_dir, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    baseline_csv = args.baseline_csv or os.path.join(results_dir, f'baseline_{timestamp}.csv')
    sitl_csv = args.sitl_csv or os.path.join(results_dir, f'sitl_{timestamp}.csv')
    html_path = os.path.join(results_dir, f'comparison_{timestamp}.html')

    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║       M130 SITL VALIDATION TEST                     ║")
    print("║       Python MPC vs C++ rocket_mpc                  ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()

    if args.compare_only:
        if not args.baseline_csv or not args.sitl_csv:
            print("ERROR: --compare-only requires --baseline-csv and --sitl-csv")
            sys.exit(1)
        compare_and_report(args.baseline_csv, args.sitl_csv, html_path)
        return

    if not args.sitl_only:
        baseline_csv = run_baseline(args.sim_config, baseline_csv)

    if args.baseline_only:
        print("\n  Baseline-only mode — done.")
        return

    if not args.baseline_only:
        sitl_csv = run_sitl(args.px4_bin, args.sitl_config,
                             args.sim_config, sitl_csv, args.port)

    if args.sitl_only and not args.baseline_csv:
        # Find latest baseline
        candidates = sorted(Path(results_dir).glob('baseline_*.csv'), reverse=True)
        if candidates:
            baseline_csv = str(candidates[0])
            print(f"\n  Using latest baseline: {baseline_csv}")
        else:
            print("\n  ERROR: No baseline CSV found. Run --baseline-only first.")
            sys.exit(1)

    if os.path.isfile(baseline_csv) and os.path.isfile(sitl_csv):
        compare_and_report(baseline_csv, sitl_csv, html_path)


if __name__ == '__main__':
    main()
