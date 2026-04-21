#!/usr/bin/env python3
"""
Monte Carlo Batch Runner for M130 6-DOF Simulation
===================================================
Runs N simulations in parallel with different error injection seeds.
Collects scalar metrics into a single CSV for statistical analysis.

Usage:
    python tools/monte_carlo_runner.py --runs 500 --workers 4
    python tools/monte_carlo_runner.py --runs 100 --workers 2 --name wind_mc \
        --overlay '{"atmosphere":{"wind_enabled":true,"wind_speed":5}}'
    python tools/monte_carlo_runner.py --runs 1000 --resume  # resume interrupted run
"""

import argparse
import copy
import json
import logging
import multiprocessing as mp
import os
import sys
import tempfile
import time
import traceback
import warnings
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


# ─── Helpers ──────────────────────────────────────────────────────────────────

def deep_update(base, overlay):
    """Recursively merge overlay into base dict."""
    for k, v in overlay.items():
        if isinstance(v, dict) and k in base and isinstance(base[k], dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base


def load_base_config(config_path=None):
    if config_path is None:
        config_path = ROOT / "config" / "6dof_config_advanced.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def extract_metrics(sim, history, run_id, seed):
    """Extract scalar metrics from a completed simulation."""
    m = {"run_id": run_id, "seed": seed, "status": "ok"}

    t_arr = np.array(history.get("time", []))
    if len(t_arr) == 0:
        m["status"] = "empty"
        return m

    pos = np.array(history.get("position", []))       # Nx3, NED
    vel = np.array(history.get("velocity", []))        # Nx3, NED
    mass = np.array(history.get("mass", []))
    alpha = np.array(history.get("alpha", []))          # rad
    beta = np.array(history.get("beta", []))            # rad
    g_force = np.array(history.get("g_force", []))
    q_dyn = np.array(history.get("q_dynamic", []))
    mach = np.array(history.get("mach", []))
    ang_vel = np.array(history.get("angular_velocity", []))  # Nx3, rad/s
    phases = history.get("flight_phase", [])

    # Ground range
    if "ground_range_km" in history and len(history["ground_range_km"]) > 0:
        ground_range_m = np.array(history["ground_range_km"]) * 1000.0
    elif pos.ndim == 2 and pos.shape[1] >= 2:
        ground_range_m = np.sqrt(pos[:, 0]**2 + pos[:, 1]**2)
    else:
        ground_range_m = np.array([0.0])

    # Altitude
    if "altitude_km" in history and len(history["altitude_km"]) > 0:
        alt_m = np.array(history["altitude_km"]) * 1000.0
    elif pos.ndim == 2 and pos.shape[1] >= 3:
        alt_m = -pos[:, 2]  # NED: z is down
    else:
        alt_m = np.array([0.0])

    # Speed
    if vel.ndim == 2 and vel.shape[1] >= 3:
        speed = np.sqrt(vel[:, 0]**2 + vel[:, 1]**2 + vel[:, 2]**2)
    else:
        speed = np.array([0.0])

    # Flight path angle at impact
    if vel.ndim == 2 and vel.shape[1] >= 3 and len(vel) > 0:
        vx, vy, vz = vel[-1, 0], vel[-1, 1], vel[-1, 2]
        vh = np.sqrt(vx**2 + vy**2)
        impact_gamma_deg = float(np.degrees(np.arctan2(-vz, vh))) if speed[-1] > 1.0 else 0.0
    else:
        impact_gamma_deg = 0.0

    # Angular rates (total, deg/s)
    if ang_vel.ndim == 2 and ang_vel.shape[1] >= 3:
        omega_total = np.degrees(np.sqrt(ang_vel[:, 0]**2 + ang_vel[:, 1]**2 + ang_vel[:, 2]**2))
    else:
        omega_total = np.array([0.0])

    # Euler angles from quaternions
    from scipy.spatial.transform import Rotation
    quat = np.array(history.get("attitude", []))
    pitch_deg = np.array([0.0])
    yaw_deg = np.array([0.0])
    if quat.ndim == 2 and quat.shape[1] == 4 and len(quat) > 0:
        try:
            r = Rotation.from_quat(quat[:, [1, 2, 3, 0]])  # scipy: [x,y,z,w]
            euler = r.as_euler("ZYX", degrees=True)  # yaw, pitch, roll
            yaw_deg = euler[:, 0]
            pitch_deg = euler[:, 1]
        except Exception:
            pass

    # ── Impact metrics ──
    m["impact_time_s"] = float(t_arr[-1])
    m["impact_range_m"] = float(ground_range_m[-1])
    m["impact_speed_mps"] = float(speed[-1])
    m["impact_gamma_deg"] = impact_gamma_deg
    m["impact_alt_m"] = float(alt_m[-1])
    m["n_steps"] = len(t_arr)
    m["impact_phase"] = str(phases[-1]) if phases else ""

    # Impact position (NED)
    if pos.ndim == 2 and pos.shape[1] >= 3:
        m["impact_x_m"] = float(pos[-1, 0])
        m["impact_y_m"] = float(pos[-1, 1])
        m["impact_z_m"] = float(pos[-1, 2])
    else:
        m["impact_x_m"] = m["impact_y_m"] = m["impact_z_m"] = 0.0

    # ── Peak / max metrics ──
    m["max_alt_m"] = float(np.max(alt_m))
    m["max_speed_mps"] = float(np.max(speed))
    m["max_mach"] = float(np.max(mach)) if len(mach) > 0 else 0.0
    m["max_q_Pa"] = float(np.max(q_dyn)) if len(q_dyn) > 0 else 0.0
    m["max_g"] = float(np.max(g_force)) if len(g_force) > 0 else 0.0
    m["max_alpha_deg"] = float(np.degrees(np.max(np.abs(alpha)))) if len(alpha) > 0 else 0.0
    m["max_beta_deg"] = float(np.degrees(np.max(np.abs(beta)))) if len(beta) > 0 else 0.0
    m["max_omega_dps"] = float(np.max(omega_total))

    # ── Stability (last 30%) ──
    n30 = max(1, int(0.3 * len(pitch_deg)))
    m["pitch_std_last30pct"] = float(np.std(pitch_deg[-n30:]))
    m["yaw_drift_deg"] = float(yaw_deg[-1] - yaw_deg[-n30]) if len(yaw_deg) > n30 else 0.0

    # ── Mass ──
    m["initial_mass_kg"] = float(mass[0]) if len(mass) > 0 else 0.0
    m["final_mass_kg"] = float(mass[-1]) if len(mass) > 0 else 0.0

    # ── NaN check ──
    nan_count = 0
    for key in ("position", "velocity", "attitude", "angular_velocity"):
        arr = np.array(history.get(key, []))
        if arr.size > 0 and np.issubdtype(arr.dtype, np.floating):
            nan_count += int(np.isnan(arr).sum())
    m["nan_count"] = nan_count

    # ── Meta ──
    meta = history.get("meta", {})
    m["impact_detected"] = meta.get("impact_detected", False)
    m["numerical_error"] = meta.get("numerical_error_termination", False)

    # ── Error injection parameters (from sim.errors) ──
    errors = getattr(sim, "errors", None) or {}
    m["err_elevation_rad"] = float(errors.get("elevation_error", 0.0))
    m["err_azimuth_rad"] = float(errors.get("azimuth_error", 0.0))
    m["err_elevation_rate_rps"] = float(errors.get("elevation_rate_error", 0.0))
    m["err_azimuth_rate_rps"] = float(errors.get("azimuth_rate_error", 0.0))
    m["err_thrust_scale"] = float(errors.get("thrust_scale_error", 1.0))
    m["err_burn_time_s"] = float(errors.get("burn_time_error", 0.0))
    m["err_mass_kg"] = float(errors.get("mass_error", 0.0))
    m["err_cg_m"] = float(errors.get("cg_error", 0.0))
    m["err_thrust_misalign_pitch_rad"] = float(errors.get("thrust_misalignment_pitch", 0.0))
    m["err_thrust_misalign_yaw_rad"] = float(errors.get("thrust_misalignment_yaw", 0.0))
    m["err_CA_scale"] = float(errors.get("CA_scale_error", 1.0))
    m["err_CN_scale"] = float(errors.get("CN_scale_error", 1.0))
    m["err_CM_scale"] = float(errors.get("CM_scale_error", 1.0))

    accel_bias = errors.get("accel_bias", np.zeros(3))
    gyro_bias = errors.get("gyro_bias", np.zeros(3))
    m["err_accel_bias_x"] = float(accel_bias[0])
    m["err_accel_bias_y"] = float(accel_bias[1])
    m["err_accel_bias_z"] = float(accel_bias[2])
    m["err_gyro_bias_x"] = float(gyro_bias[0])
    m["err_gyro_bias_y"] = float(gyro_bias[1])
    m["err_gyro_bias_z"] = float(gyro_bias[2])

    return m


def run_single(args):
    """Worker function for multiprocessing. Returns metrics dict."""
    run_id, seed, config_path_str, overlay_dict = args

    # Suppress matplotlib GUI in worker processes
    import matplotlib
    matplotlib.use("Agg")

    # Suppress sim logging noise
    logging.disable(logging.WARNING)
    warnings.filterwarnings("ignore")

    result = {"run_id": run_id, "seed": seed, "status": "crashed"}
    tmp_path = None

    try:
        cfg = load_base_config(config_path_str)
        if overlay_dict:
            deep_update(cfg, overlay_dict)

        # Enable error injection with this seed
        cfg.setdefault("simulation", {})["use_error_injection"] = True
        cfg.setdefault("error_injection", {})["seed"] = seed

        # Disable browser and auto-analysis
        cfg["simulation"]["auto_analyze"] = False

        # Write temp config
        tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, encoding="utf-8"
        )
        yaml.dump(cfg, tmp, allow_unicode=True, default_flow_style=False)
        tmp.close()
        tmp_path = tmp.name

        from rocket_6dof_sim import Rocket6DOFSimulation

        sim = Rocket6DOFSimulation(config_file=tmp_path, long_range_mode=False)
        sim_cfg = cfg.get("simulation", {})
        duration = sim_cfg.get("duration", 300.0)
        dt = sim_cfg.get("dt", 0.01)

        # Use configured control type (MPC or ballistic)
        control_type = sim_cfg.get("control_type", "ballistic")
        if control_type == "ballistic":
            ballistic_fin = np.array(sim_cfg.get("ballistic_fin_deflection", [0, 0, 0, 0]))
            ctrl = lambda state, t: ballistic_fin.copy()
        else:
            ctrl = None  # Let MPC handle it

        history = sim.simulate(duration=duration, dt=dt, control_function=ctrl)
        result = extract_metrics(sim, history, run_id, seed)

    except Exception as e:
        result["error_msg"] = f"{type(e).__name__}: {str(e)[:200]}"
        result["status"] = "crashed"
    finally:
        logging.disable(logging.NOTSET)
        if tmp_path and os.path.exists(tmp_path):
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

    return result


class MonteCarloRunner:
    """Batch Monte Carlo execution engine with parallel workers."""

    def __init__(self, config_path=None, n_runs=100, n_workers=None,
                 output_dir=None, run_name=None, base_seed=0, overlay=None):
        self.config_path = str(config_path or ROOT / "config" / "6dof_config_advanced.yaml")
        self.n_runs = n_runs
        self.n_workers = n_workers or max(1, os.cpu_count() - 1)
        self.base_seed = base_seed
        self.overlay = overlay or {}
        self.run_name = run_name or f"mc_{n_runs}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        if output_dir:
            self.output_dir = Path(output_dir)
        else:
            self.output_dir = ROOT / "results" / "monte_carlo"
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.metrics_csv = self.output_dir / f"{self.run_name}_metrics.csv"
        self.summary_json = self.output_dir / f"{self.run_name}_summary.json"

    def _get_completed_run_ids(self):
        """Load existing CSV and return set of completed run_ids."""
        if not self.metrics_csv.exists():
            return set()
        try:
            import pandas as pd
            df = pd.read_csv(self.metrics_csv)
            return set(df["run_id"].values)
        except Exception:
            return set()

    def run(self):
        """Execute all MC runs with multiprocessing."""
        import pandas as pd

        completed = self._get_completed_run_ids()
        remaining = [(i, self.base_seed + i, self.config_path, self.overlay)
                      for i in range(self.n_runs) if i not in completed]

        if not remaining:
            print(f"  All {self.n_runs} runs already completed. Loading results.")
            return pd.read_csv(self.metrics_csv)

        n_done = self.n_runs - len(remaining)
        print(f"  Monte Carlo: {len(remaining)} runs to execute "
              f"({n_done} already done), {self.n_workers} workers")
        print(f"  Output: {self.metrics_csv}")

        all_results = []

        # Load existing results
        if completed and self.metrics_csv.exists():
            existing_df = pd.read_csv(self.metrics_csv)
            all_results = existing_df.to_dict("records")

        t0 = time.time()
        batch = []
        batch_size = 50  # incremental save every 50 runs

        try:
            from tqdm import tqdm
            progress = tqdm(total=len(remaining), desc="MC runs", unit="run",
                            initial=0, ncols=80)
        except ImportError:
            progress = None
            print("  (install tqdm for progress bar: pip install tqdm)")

        with mp.Pool(processes=self.n_workers) as pool:
            for result in pool.imap_unordered(run_single, remaining):
                all_results.append(result)
                batch.append(result)

                if progress:
                    status = result.get("status", "?")
                    rng = result.get("impact_range_m", 0)
                    progress.set_postfix_str(f"#{result['run_id']} {status} R={rng:.0f}m")
                    progress.update(1)

                # Incremental save
                if len(batch) >= batch_size:
                    self._save(all_results)
                    batch = []

        if progress:
            progress.close()

        # Final save
        df = self._save(all_results)

        elapsed = time.time() - t0
        ok = df[df["status"] == "ok"]
        crashed = df[df["status"] == "crashed"]

        # Summary
        summary = {
            "run_name": self.run_name,
            "n_runs": self.n_runs,
            "n_ok": len(ok),
            "n_crashed": len(crashed),
            "elapsed_s": round(elapsed, 1),
            "runs_per_sec": round(len(remaining) / elapsed, 2) if elapsed > 0 else 0,
            "config": self.config_path,
            "timestamp": datetime.now().isoformat(),
        }
        if len(ok) > 0:
            summary["range_mean_m"] = round(float(ok["impact_range_m"].mean()), 1)
            summary["range_std_m"] = round(float(ok["impact_range_m"].std()), 1)

        with open(self.summary_json, "w") as f:
            json.dump(summary, f, indent=2)

        print(f"\n  Done: {len(ok)} ok, {len(crashed)} crashed "
              f"in {elapsed:.1f}s ({summary['runs_per_sec']:.1f} runs/s)")
        if len(ok) > 0:
            print(f"  Range: {summary['range_mean_m']:.0f} ± {summary['range_std_m']:.0f} m")
        print(f"  CSV: {self.metrics_csv}")
        print(f"  Summary: {self.summary_json}")

        return df

    def _save(self, results):
        """Save results list to CSV."""
        import pandas as pd
        df = pd.DataFrame(results)
        df.to_csv(self.metrics_csv, index=False)
        return df


def main():
    parser = argparse.ArgumentParser(
        description="Monte Carlo Runner for M130 6-DOF Simulation")
    parser.add_argument("--runs", type=int, default=100,
                        help="Number of MC runs (default: 100)")
    parser.add_argument("--workers", type=int, default=None,
                        help="Number of parallel workers (default: cpu_count-1)")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to base config YAML")
    parser.add_argument("--name", type=str, default=None,
                        help="Run name prefix for output files")
    parser.add_argument("--base-seed", type=int, default=0,
                        help="Starting seed (run i uses seed = base_seed + i)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory")
    parser.add_argument("--overlay", type=str, default=None,
                        help='JSON config overlay, e.g. \'{"atmosphere":{"wind_speed":5}}\'')
    parser.add_argument("--resume", action="store_true",
                        help="Resume interrupted run (requires --name)")
    args = parser.parse_args()

    overlay = {}
    if args.overlay:
        overlay = json.loads(args.overlay)

    runner = MonteCarloRunner(
        config_path=args.config,
        n_runs=args.runs,
        n_workers=args.workers,
        output_dir=args.output_dir,
        run_name=args.name,
        base_seed=args.base_seed,
        overlay=overlay,
    )
    runner.run()


if __name__ == "__main__":
    main()
