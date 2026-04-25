#!/usr/bin/env python3
"""
════════════════════════════════════════════════════════════════════════
  M130 HIL Flight Analysis — Interactive HTML Report
  تحليل نتائج طيران HIL (Hardware-In-the-Loop) — تقرير HTML تفاعلي
════════════════════════════════════════════════════════════════════════

Generates a self-contained professional HTML report for HIL flights.
Based on pil_analysis.py with HIL-specific additions:
  - Servo CAN feedback (fin_can_1-4) vs command tracking
  - online_mask / tx_fail CAN health
  - fin_source breakdown (can / hold / cmd / abort)
  - Servo hardware scoring (10 pts — tracking MAE, dropout rate)
  - Dedicated Servo Hardware and CAN Health tabs
  - Additional Timing tab if hil_timing.csv has samples

Usage:
    python hil_analysis.py                        # latest HIL CSV
    python hil_analysis.py --file <csv_path>      # specific file
    python hil_analysis.py --no-open              # don't open browser
"""

import sys
import argparse
import webbrowser
from pathlib import Path
from datetime import datetime
from html import escape as html_escape

import numpy as np
import pandas as pd
import yaml

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    import plotly.io as pio
except ImportError:
    print("ERROR: plotly is required.  pip install plotly")
    sys.exit(1)

# ─── Paths ────────────────────────────────────────────────────────────
_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS_DIR = _SCRIPT_DIR / "results"
_CONFIG_PATH = _SIM_DIR / "config" / "6dof_config_advanced.yaml"
_HIL_CONFIG_PATH = _SCRIPT_DIR / "hil_config.yaml"

# ─── Mission Config ───────────────────────────────────────────────────
try:
    with open(_CONFIG_PATH, "r") as _f:
        _cfg = yaml.safe_load(_f)
    TARGET_RANGE_M = float(_cfg.get("target", {}).get("range_m", 2900.0))
    LAUNCH_ALT_M = float(_cfg.get("launch", {}).get("altitude", 1200.0))
except Exception:
    TARGET_RANGE_M = 2900.0
    LAUNCH_ALT_M = 1200.0

# ─── HIL Timing & Servo Thresholds ────────────────────────────────────
try:
    with open(_HIL_CONFIG_PATH, "r") as _f:
        _hil_cfg = yaml.safe_load(_f)
    TIMING_DEADLINE_US = int(_hil_cfg.get("timing", {}).get("deadline_us", 20000))
    _srv_thresh = _hil_cfg.get("thresholds", {}).get("servo_tracking", {})
    SERVO_MAE_MAX_DEG = float(_srv_thresh.get("tracking_mae_max_deg", 1.5))
    SERVO_P95_MAX_DEG = float(_srv_thresh.get("tracking_p95_max_deg", 3.0))
except Exception:
    TIMING_DEADLINE_US = 20000
    SERVO_MAE_MAX_DEG = 1.5
    SERVO_P95_MAX_DEG = 3.0
TIMING_WARN_US = 10000


# ══════════════════════════════════════════════════════════════════════
#  Data Loading & Derived Quantities
# ══════════════════════════════════════════════════════════════════════

def load_hil_csv(path: Path) -> pd.DataFrame:
    """Load HIL bridge CSV and compute derived columns.

    HIL CSV has extra columns vs PIL: fin_can_1-4 (rad), fin_source.
    Like PIL, lacks accel_x/y/z — derived from velocity gradient.
    """
    df = pd.read_csv(path)
    df.rename(columns={"time": "time_s"}, inplace=True)

    # Speed
    df["speed_total"] = np.sqrt(df["vel_x"]**2 + df["vel_y"]**2 + df["vel_z"]**2)
    df["speed_horizontal"] = np.sqrt(df["vel_x"]**2 + df["vel_y"]**2)

    # Angles in degrees
    df["alpha_deg"] = np.degrees(df["alpha"])
    df["beta_deg"] = np.degrees(df["beta"])

    # Euler angles from quaternion
    q0, q1, q2, q3 = df["q0"], df["q1"], df["q2"], df["q3"]
    df["roll_deg"] = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
    df["pitch_deg"] = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
    df["yaw_deg"] = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

    # Flight path angle from altitude change
    with np.errstate(divide='ignore', invalid='ignore'):
        v_h = df["speed_horizontal"]
        dt = np.gradient(df["time_s"].values)
        dt = np.where(dt > 0, dt, 1e-6)
        dalt = np.gradient(df["altitude"].values)
        climb_rate = dalt / dt
        df["gamma_deg"] = np.degrees(np.arctan2(climb_rate, np.maximum(v_h, 0.1)))

    # Angular rates in deg/s
    df["omega_x_deg"] = np.degrees(df["omega_x"])
    df["omega_y_deg"] = np.degrees(df["omega_y"])
    df["omega_z_deg"] = np.degrees(df["omega_z"])
    df["omega_total_deg"] = np.sqrt(df["omega_x_deg"]**2 + df["omega_y_deg"]**2 + df["omega_z_deg"]**2)

    # Fin deflections in degrees (command & actuator model)
    for i in range(1, 5):
        df[f"fin_cmd_{i}_deg"] = np.degrees(df[f"fin_cmd_{i}"])
        df[f"fin_act_{i}_deg"] = np.degrees(df[f"fin_act_{i}"])
        df[f"fin_lag_{i}_deg"] = df[f"fin_cmd_{i}_deg"] - df[f"fin_act_{i}_deg"]

    # HIL-specific: CAN feedback in degrees + tracking error
    if all(f"fin_can_{i}" in df.columns for i in range(1, 5)):
        for i in range(1, 5):
            df[f"fin_can_{i}_deg"] = np.degrees(df[f"fin_can_{i}"])
            df[f"fin_can_err_{i}_deg"] = df[f"fin_cmd_{i}_deg"] - df[f"fin_can_{i}_deg"]

    # HIL CSV lacks accel_* — derive from velocity gradient
    if "accel_x" not in df.columns:
        dt_grad = np.gradient(df["time_s"].values)
        dt_grad = np.where(dt_grad > 0, dt_grad, 1e-6)
        df["accel_x"] = np.gradient(df["vel_x"].values) / dt_grad
        df["accel_y"] = np.gradient(df["vel_y"].values) / dt_grad
        df["accel_z"] = np.gradient(df["vel_z"].values) / dt_grad

    # G-load from (derived) body accelerations
    df["g_total"] = np.sqrt(df["accel_x"]**2 + df["accel_y"]**2 + df["accel_z"]**2) / 9.80665
    df["g_axial"] = df["accel_x"] / 9.80665

    # Force magnitude
    df["force_total"] = np.sqrt(df["force_x"]**2 + df["force_y"]**2 + df["force_z"]**2)

    # Energy
    df["KE_kJ"] = 0.5 * df["mass"] * df["speed_total"]**2 / 1000
    df["PE_kJ"] = df["mass"] * 9.81 * df["altitude"] / 1000
    df["total_energy_kJ"] = df["KE_kJ"] + df["PE_kJ"]

    # Alt AGL (subtract launch altitude)
    df["alt_agl_m"] = df["altitude"] - LAUNCH_ALT_M

    # Flight phase detection
    df["flight_phase"] = "COAST"
    dm = np.gradient(df["mass"].values)
    df.loc[dm < -0.01, "flight_phase"] = "BOOST"
    if df["alt_agl_m"].max() > 0:
        df.loc[(df["alt_agl_m"] < df["alt_agl_m"].max() * 0.3) &
               (df["time_s"] > df["time_s"].iloc[-1] * 0.7), "flight_phase"] = "TERMINAL"

    return df


def load_hil_timing(csv_path: Path):
    """Load hil_timing.csv if it has samples.  Returns None if empty/missing."""
    timing_path = csv_path.parent / "hil_timing.csv"
    if not timing_path.exists():
        return None
    try:
        tdf = pd.read_csv(timing_path)
    except Exception:
        return None
    if len(tdf) == 0:
        return None
    return tdf


def load_hil_servos(csv_path: Path):
    """Load HIL servo feedback CSV (CAN bus log).  Returns None if missing."""
    # The bridge saves servo CSV as <flight_csv_stem>_servo.csv
    servo_path = csv_path.with_name(csv_path.stem + "_servo.csv")
    if not servo_path.exists():
        servo_path = csv_path.parent / "hil_servos.csv"
    if not servo_path.exists():
        return None
    try:
        sdf = pd.read_csv(servo_path)
    except Exception:
        return None
    if len(sdf) == 0:
        return None
    return sdf


# ══════════════════════════════════════════════════════════════════════
#  Metrics Extraction
# ══════════════════════════════════════════════════════════════════════

def extract_metrics(df: pd.DataFrame, filepath: Path,
                    timing_df=None, servo_df=None) -> dict:
    m = {}
    t = df["time_s"].values
    last = df.iloc[-1]

    m["file"] = filepath.name
    m["timestamp"] = filepath.stem
    m["flight_time_s"] = t[-1]
    m["n_steps"] = len(df)

    m["impact_range_m"] = last["ground_range"]
    m["range_error_m"] = last["ground_range"] - TARGET_RANGE_M
    m["range_error_pct"] = (last["ground_range"] / TARGET_RANGE_M - 1) * 100 if TARGET_RANGE_M > 0 else 0

    m["peak_alt_m"] = df["alt_agl_m"].max()
    m["peak_alt_time_s"] = df.loc[df["alt_agl_m"].idxmax(), "time_s"]
    m["impact_speed_mps"] = last["speed_total"]
    m["max_speed_mps"] = df["speed_total"].max()
    m["max_mach"] = df["mach"].max()

    m["max_alpha_deg"] = df["alpha_deg"].abs().max()
    m["max_beta_deg"] = df["beta_deg"].abs().max()
    m["max_g"] = df["g_total"].max()
    m["max_axial_g"] = df["g_axial"].max()

    m["max_omega_deg_s"] = df["omega_total_deg"].max()

    m["max_fin_cmd_deg"] = max(df[f"fin_cmd_{i}_deg"].abs().max() for i in range(1, 5))
    m["max_fin_act_deg"] = max(df[f"fin_act_{i}_deg"].abs().max() for i in range(1, 5))

    m["mass_initial"] = df["mass"].iloc[0]
    m["mass_final"] = df["mass"].iloc[-1]
    m["propellant_kg"] = df["mass"].iloc[0] - df["mass"].iloc[-1]

    n30 = max(1, int(len(df) * 0.3))
    m["pitch_std_last30pct"] = df["pitch_deg"].iloc[-n30:].std()

    m["impact_gamma_deg"] = df["gamma_deg"].iloc[-1]
    m["impact_pitch_deg"] = last["pitch_deg"]

    # Max |alpha| during stable flight (exclude terminal tumble in last 5%)
    t_cutoff = t[-1] * 0.95
    stable = df[df["time_s"] < t_cutoff]
    m["max_alpha_flight_deg"] = stable["alpha_deg"].abs().max() if len(stable) else m["max_alpha_deg"]

    # Fin saturation during stable flight
    sat_frac = 0.0
    if len(stable):
        n_stable = len(stable)
        for i in range(1, 5):
            f_sat = (stable[f"fin_cmd_{i}_deg"].abs() > 19.5).sum() / n_stable
            sat_frac = max(sat_frac, f_sat)
    m["fin_saturation_pct"] = sat_frac * 100

    # ─── HIL-specific: fin_source breakdown ───────────────────────────
    if "fin_source" in df.columns:
        src_counts = df["fin_source"].value_counts()
        total_src = len(df)
        for src in ["can", "hold", "cmd", "abort"]:
            m[f"fin_source_{src}_pct"] = float(src_counts.get(src, 0)) / total_src * 100
            m[f"fin_source_{src}_n"] = int(src_counts.get(src, 0))
    else:
        for src in ["can", "hold", "cmd", "abort"]:
            m[f"fin_source_{src}_pct"] = 0.0
            m[f"fin_source_{src}_n"] = 0

    # ─── HIL-specific: CAN tracking error from flight CSV ─────────────
    has_can_cols = all(f"fin_can_err_{i}_deg" in df.columns for i in range(1, 5))
    if has_can_cols and len(stable):
        can_mask = df["fin_source"] == "can" if "fin_source" in df.columns else pd.Series(True, index=df.index)
        can_stable = stable[can_mask.loc[stable.index]]
        if len(can_stable) > 10:
            all_err = np.concatenate([can_stable[f"fin_can_err_{i}_deg"].abs().values for i in range(1, 5)])
            m["servo_tracking_mae_deg"] = float(np.mean(all_err))
            m["servo_tracking_p95_deg"] = float(np.percentile(all_err, 95))
            m["servo_tracking_max_deg"] = float(np.max(all_err))
            m["servo_over_2deg_pct"] = float((all_err > 2.0).mean() * 100)
        else:
            m["servo_tracking_mae_deg"] = 0.0
            m["servo_tracking_p95_deg"] = 0.0
            m["servo_tracking_max_deg"] = 0.0
            m["servo_over_2deg_pct"] = 0.0
    else:
        m["servo_tracking_mae_deg"] = 0.0
        m["servo_tracking_p95_deg"] = 0.0
        m["servo_tracking_max_deg"] = 0.0
        m["servo_over_2deg_pct"] = 0.0

    # ─── HIL-specific: servo CSV metrics (online_mask, tx_fail) ───────
    if servo_df is not None and len(servo_df) > 0:
        m["servo_samples"] = len(servo_df)
        if "online_mask" in servo_df.columns:
            masks = servo_df["online_mask"].values.astype(int)
            m["servo_all_online_pct"] = float((masks == 0x0F).mean() * 100)
            for ch in range(4):
                m[f"servo_{ch+1}_online_pct"] = float(((masks >> ch) & 1).mean() * 100)
        else:
            m["servo_all_online_pct"] = 0.0
            for ch in range(4):
                m[f"servo_{ch+1}_online_pct"] = 0.0
        if "tx_fail" in servo_df.columns:
            m["servo_tx_fail_total"] = int(servo_df["tx_fail"].max())
        else:
            m["servo_tx_fail_total"] = 0
    else:
        m["servo_samples"] = 0
        m["servo_all_online_pct"] = 0.0
        for ch in range(4):
            m[f"servo_{ch+1}_online_pct"] = 0.0
        m["servo_tx_fail_total"] = 0

    # Timing metrics
    if timing_df is not None and len(timing_df) > 0:
        m["timing_samples"] = len(timing_df)
        for col, label in [("mpc_us", "mpc"), ("mhe_us", "mhe"), ("cycle_us", "cycle")]:
            if col in timing_df.columns:
                m[f"{label}_us_avg"] = float(timing_df[col].mean())
                m[f"{label}_us_p99"] = float(timing_df[col].quantile(0.99))
                m[f"{label}_us_max"] = float(timing_df[col].max())
                m[f"{label}_over_deadline_pct"] = float((timing_df[col] > TIMING_DEADLINE_US).mean() * 100)
    else:
        m["timing_samples"] = 0

    return m


# ══════════════════════════════════════════════════════════════════════
#  Scoring (rebalanced for HIL — adds servo_hw weight)
# ══════════════════════════════════════════════════════════════════════

SCORE_WEIGHTS = {"range": 25, "impact_angle": 15, "stability": 15, "aoa": 15, "sideslip": 10, "g_load": 10, "servo_hw": 10}
THRESH = {
    "range_error_good":  50,
    "range_error_warn":  200,
    "max_alpha_warn":    15,
    "max_alpha_fail":    25,
    "max_beta_warn":     5,
    "max_beta_fail":     10,
    "pitch_std_warn":    3,
    "pitch_std_fail":    8,
    "max_g_warn":        15,
    "max_g_fail":        25,
}


def score_run(m: dict) -> dict:
    scores = {}
    W = SCORE_WEIGHTS

    # Range
    w = W["range"]
    abs_err = abs(m["range_error_m"])
    if abs_err < THRESH["range_error_good"]:
        s, v = w, "PASS"
    elif abs_err < THRESH["range_error_warn"]:
        s = w * (1 - (abs_err - THRESH["range_error_good"]) /
                     (THRESH["range_error_warn"] - THRESH["range_error_good"]))
        v = "WARN"
    else:
        s = max(0, (w * 0.25) * (1 - abs_err / 1000))
        v = "FAIL"
    scores["range"] = {"score": round(s, 1), "verdict": v,
                        "detail": f"|err| = {abs_err:.0f}m ({m['range_error_pct']:+.1f}%)"}

    # Impact angle (adaptive)
    w = W["impact_angle"]
    gamma = m["impact_gamma_deg"]
    is_flat = m["peak_alt_m"] < 500
    if is_flat:
        if gamma < -3:
            s, v = w, "PASS"
        elif gamma < 0:
            s, v = round(w * 0.8, 1), "PASS"
        else:
            s, v = round(w * 0.4, 1), "WARN"
    else:
        if -80 < gamma < -20:
            s, v = w, "PASS"
        elif -90 < gamma < -10:
            s, v = round(w * 0.53, 1), "WARN"
        else:
            s, v = 0, "FAIL"
    traj_type = "flat" if is_flat else "steep"
    scores["impact_angle"] = {"score": round(s, 1), "verdict": v,
                               "detail": f"γ = {gamma:.1f}° ({traj_type} trajectory)"}

    # Stability
    w = W["stability"]
    pstd = m["pitch_std_last30pct"]
    if pstd < THRESH["pitch_std_warn"]:
        s, v = w, "PASS"
    elif pstd < THRESH["pitch_std_fail"]:
        s = w * (1 - (pstd - THRESH["pitch_std_warn"]) /
                     (THRESH["pitch_std_fail"] - THRESH["pitch_std_warn"]))
        v = "WARN"
    else:
        s, v = 0, "FAIL"
    scores["stability"] = {"score": round(s, 1), "verdict": v,
                            "detail": f"pitch σ = {pstd:.2f}°"}

    # AoA — use during-flight value (not terminal)
    w = W["aoa"]
    alpha = m.get("max_alpha_flight_deg", m["max_alpha_deg"])
    if alpha < THRESH["max_alpha_warn"]:
        s, v = w, "PASS"
    elif alpha < THRESH["max_alpha_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["aoa"] = {"score": round(s, 1), "verdict": v,
                      "detail": f"max |α| (flight) = {alpha:.1f}°"}

    # Sideslip
    w = W["sideslip"]
    beta = m["max_beta_deg"]
    if beta < THRESH["max_beta_warn"]:
        s, v = w, "PASS"
    elif beta < THRESH["max_beta_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["sideslip"] = {"score": round(s, 1), "verdict": v,
                           "detail": f"max |β| = {beta:.1f}°"}

    # G-load
    w = W["g_load"]
    g = m["max_g"]
    if g < THRESH["max_g_warn"]:
        s, v = w, "PASS"
    elif g < THRESH["max_g_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["g_load"] = {"score": round(s, 1), "verdict": v,
                         "detail": f"max G = {g:.1f}"}

    # ─── Servo hardware (HIL-specific) ────────────────────────────────
    w = W["servo_hw"]
    mae = m.get("servo_tracking_mae_deg", 0.0)
    p95 = m.get("servo_tracking_p95_deg", 0.0)
    can_pct = m.get("fin_source_can_pct", 0.0)
    online_pct = m.get("servo_all_online_pct", 0.0)

    if mae == 0.0 and m.get("servo_samples", 0) == 0 and m.get("fin_source_can_n", 0) == 0:
        s, v = round(w * 0.5, 1), "WARN"
        servo_detail = "no servo feedback data"
    else:
        s_mae = 1.0 if mae < SERVO_MAE_MAX_DEG else (0.5 if mae < SERVO_MAE_MAX_DEG * 2 else 0.0)
        s_p95 = 1.0 if p95 < SERVO_P95_MAX_DEG else (0.5 if p95 < SERVO_P95_MAX_DEG * 2 else 0.0)
        s_can = min(1.0, can_pct / 90.0)
        s_online = min(1.0, online_pct / 95.0)
        total_sub = 0.4 * s_mae + 0.3 * s_p95 + 0.15 * s_can + 0.15 * s_online
        s = round(w * total_sub, 1)
        if total_sub >= 0.8:
            v = "PASS"
        elif total_sub >= 0.4:
            v = "WARN"
        else:
            v = "FAIL"
        servo_detail = (f"MAE={mae:.2f}° P95={p95:.2f}° "
                        f"CAN={can_pct:.0f}% online={online_pct:.0f}%")
    scores["servo_hw"] = {"score": round(s, 1), "verdict": v,
                           "detail": servo_detail}

    total = sum(c["score"] for c in scores.values())
    overall = "PASS" if total >= 80 else ("WARN" if total >= 50 else "FAIL")
    scores["_total"] = round(total, 1)
    scores["_overall"] = overall

    checks = [(k, v["verdict"], v["detail"]) for k, v in scores.items() if not k.startswith("_")]
    return {"checks": checks,
            "details": {k: v["score"] for k, v in scores.items() if not k.startswith("_")},
            "overall": overall, "total": round(total, 1)}


# ══════════════════════════════════════════════════════════════════════
#  Diagnostics & Recommendations (HIL-specific)
# ══════════════════════════════════════════════════════════════════════

def diagnose(df, m):
    diags = []
    if m["flight_time_s"] < 5:
        diags.append(("error", "Very Short Flight",
                      f"Only {m['flight_time_s']:.1f}s — possible early crash or sim failure"))
    if m.get("max_alpha_flight_deg", m["max_alpha_deg"]) > 15:
        diags.append(("warning", "High Angle of Attack",
                      f"|α| reached {m['max_alpha_flight_deg']:.1f}° during flight "
                      f"— may indicate MPC solver timeouts on ARM64"))
    if m["max_omega_deg_s"] > 200:
        diags.append(("warning", "High Angular Rate",
                      f"ω reached {m['max_omega_deg_s']:.0f}°/s — tumbling risk"))
    if m["max_fin_cmd_deg"] < 0.1:
        diags.append(("warning", "No Control Commands",
                      "PX4 rocket_mpc sent near-zero fin commands — check ARM state"))
    if m["fin_saturation_pct"] > 3:
        diags.append(("warning", "Fin Saturation",
                      f"Fins at max deflection {m['fin_saturation_pct']:.1f}% of flight "
                      f"— solver may be computing unstable controls"))
    if m["propellant_kg"] < 0.1:
        diags.append(("info", "No Propellant Burned",
                      "Mass barely changed — check thrust model"))

    # ─── HIL servo diagnostics ────────────────────────────────────────
    if m.get("servo_samples", 0) == 0 and m.get("fin_source_can_pct", 0) == 0:
        diags.append(("warning", "No Servo Feedback Data",
                      "No SRV_FB frames received — check CAN bus wiring, "
                      "XqpowerCan driver, and DEBUG_FLOAT_ARRAY id=1 stream"))
    else:
        mae = m.get("servo_tracking_mae_deg", 0)
        if mae > SERVO_MAE_MAX_DEG * 2:
            diags.append(("error", "Poor Servo Tracking",
                          f"MAE = {mae:.2f}° (limit {SERVO_MAE_MAX_DEG}°) "
                          f"— check servo calibration, backlash, CAN latency"))
        elif mae > SERVO_MAE_MAX_DEG:
            diags.append(("warning", "Servo Tracking Degraded",
                          f"MAE = {mae:.2f}° (limit {SERVO_MAE_MAX_DEG}°) "
                          f"— marginal; may affect precision"))
        over_2 = m.get("servo_over_2deg_pct", 0)
        if over_2 > 10:
            diags.append(("warning", "Frequent Large Tracking Errors",
                          f"{over_2:.1f}% of samples have |err| > 2° "
                          f"— servo slew rate or CAN jitter issue"))
        hold_pct = m.get("fin_source_hold_pct", 0)
        abort_n = m.get("fin_source_abort_n", 0)
        cmd_pct = m.get("fin_source_cmd_pct", 0)
        if abort_n > 0:
            diags.append(("error", "Servo Feedback Abort Triggered",
                          f"Simulation aborted {abort_n} step(s) due to "
                          f"feedback loss — CAN bus failure"))
        if hold_pct > 5:
            diags.append(("warning", "Servo Feedback Stale Periods",
                          f"{hold_pct:.1f}% of steps used last-known CAN angle "
                          f"(stale feedback) — intermittent CAN dropouts"))
        if cmd_pct > 10:
            diags.append(("warning", "Extended Command Fallback",
                          f"{cmd_pct:.1f}% of steps used MPC command as fin angle "
                          f"(no CAN feedback) — check grace period / startup"))
        online_pct = m.get("servo_all_online_pct", 0)
        if online_pct < 90:
            diags.append(("warning", "Servo Offline Events",
                          f"All 4 servos online only {online_pct:.0f}% of time "
                          f"— check CAN bus connections"))
        tx_fail = m.get("servo_tx_fail_total", 0)
        if tx_fail > 10:
            diags.append(("warning", "CAN TX Failures",
                          f"{tx_fail} TX failures reported — "
                          f"bus congestion or wiring issue"))

    # Timing diagnostics
    if m["timing_samples"] == 0:
        diags.append(("info", "No Timing Data",
                      "hil_timing.csv is empty — 'adb reverse tcp:5760 tcp:5760' "
                      "may not be set, or MAVLink DEBUG_VECT stream is not enabled"))
    else:
        over_pct = m.get("mpc_over_deadline_pct", 0)
        if over_pct > 50:
            diags.append(("error", "MPC Deadline Violations",
                          f"mpc_us exceeded {TIMING_DEADLINE_US}us in {over_pct:.0f}% of cycles "
                          f"— ARM64 cannot meet realtime budget"))
        elif over_pct > 10:
            diags.append(("warning", "Occasional MPC Timeouts",
                          f"mpc_us exceeded deadline in {over_pct:.0f}% of cycles"))

    return diags


def recommend(m, scores, diags):
    recs = []
    if m["max_fin_cmd_deg"] < 0.5:
        recs.append("Verify PX4 is armed and rocket_mpc module is running — fin commands are near-zero")
    if abs(m["range_error_pct"]) > 15:
        recs.append(f"Range error is {m['range_error_pct']:+.1f}% — tune MPC guidance gains or verify thrust curve")
    if m.get("max_alpha_flight_deg", 0) > 12:
        recs.append("High in-flight AoA — consider lowering qp_solver_cond_N further or reducing N_horizon")
    if m["pitch_std_last30pct"] > 5:
        recs.append("Pitch oscillation in terminal phase — check damping gains and actuator bandwidth")
    if m["fin_saturation_pct"] > 3:
        recs.append("Fins saturating — likely MPC solve timeouts produce suboptimal controls; "
                    "check qp_solver_cond_N is set (should be ~10, not None)")
    # HIL servo recommendations
    mae = m.get("servo_tracking_mae_deg", 0)
    if mae > SERVO_MAE_MAX_DEG:
        recs.append(f"Servo tracking MAE={mae:.2f}° exceeds {SERVO_MAE_MAX_DEG}° limit — "
                    "check fin mechanical play, servo_auto_zero calibration, CAN bus speed")
    if m.get("fin_source_hold_pct", 0) > 5:
        recs.append("Significant CAN dropout — check wiring, connector seating, "
                    "servo_feedback_timeout_ms in hil_config.yaml")
    if m.get("servo_all_online_pct", 0) < 90 and m.get("servo_samples", 0) > 0:
        for ch in range(4):
            pct = m.get(f"servo_{ch+1}_online_pct", 100)
            if pct < 80:
                recs.append(f"Servo {ch+1} online only {pct:.0f}% — "
                            f"check CAN address and power to fin {ch+1}")
    if m.get("servo_tx_fail_total", 0) > 10:
        recs.append("CAN TX failures detected — check bus termination resistors and cable length")
    if m["timing_samples"] == 0:
        recs.append("Enable timing capture: ensure 'adb reverse tcp:5760 tcp:5760' and PX4 publishes "
                    "DEBUG_VECT 'TIMING' with fields mhe_us/mpc_us/cycle_us")
    if not diags and scores["overall"] == "PASS":
        recs.append("HIL flight nominal — no issues detected")
    return recs


# ══════════════════════════════════════════════════════════════════════
#  Plotly Figures
# ══════════════════════════════════════════════════════════════════════

def _fig_trajectory(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Altitude AGL vs Time", "Ground Range vs Time",
                                        "Trajectory Profile", "Speed vs Time"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["alt_agl_m"], mode="lines",
                             name="Altitude", line=dict(color="#1f77b4", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["ground_range"], mode="lines",
                             name="Range", line=dict(color="#ff7f0e", width=2)), row=1, col=2)
    fig.add_trace(go.Scatter(x=df["ground_range"], y=df["alt_agl_m"], mode="lines",
                             name="Profile", line=dict(color="#2ca02c", width=2)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["speed_total"], mode="lines",
                             name="Speed", line=dict(color="#d62728", width=2)), row=2, col=2)
    fig.add_hline(y=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.5, row=1, col=2,
                  annotation_text=f"Target {TARGET_RANGE_M:.0f}m")
    fig.add_vline(x=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.4, row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Ground Range (m)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Altitude AGL (m)", row=1, col=1)
    fig.update_yaxes(title_text="Range (m)", row=1, col=2)
    fig.update_yaxes(title_text="Altitude AGL (m)", row=2, col=1)
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white",
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig


def _fig_attitude(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Euler Angles", "Flight Path γ",
                                        "α & β (AoA / Sideslip)", "Angular Rates"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["pitch_deg"], name="Pitch", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["yaw_deg"], name="Yaw", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["roll_deg"], name="Roll", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["gamma_deg"], name="γ",
                             line=dict(color="#17becf"), showlegend=False), row=1, col=2)
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.3, row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["alpha_deg"], name="α",
                             line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["beta_deg"], name="β",
                             line=dict(color="#d62728"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["omega_y_deg"], name="q (pitch)",
                             line=dict(color="#1f77b4"), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["omega_z_deg"], name="r (yaw)",
                             line=dict(color="#d62728"), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["omega_x_deg"], name="p (roll)",
                             line=dict(color="#2ca02c"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Angle (°)", row=1, col=1)
    fig.update_yaxes(title_text="γ (°)", row=1, col=2)
    fig.update_yaxes(title_text="Angle (°)", row=2, col=1)
    fig.update_yaxes(title_text="Rate (°/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_aero(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Mach Number", "G-Load",
                                        "α vs Mach", "Mass & Propellant"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["mach"], name="Mach",
                             line=dict(color="#9467bd", width=2)), row=1, col=1)
    fig.add_hline(y=1.0, line_dash="dash", line_color="red", opacity=0.4, row=1, col=1,
                  annotation_text="Mach 1")
    fig.add_trace(go.Scatter(x=t, y=df["g_total"], name="G-Load",
                             line=dict(color="#e74c3c", width=2)), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["g_axial"], name="Axial G",
                             line=dict(color="#ff7f0e", width=1, dash="dot")), row=1, col=2)
    fig.add_trace(go.Scatter(x=df["mach"], y=df["alpha_deg"], mode="markers",
                             name="α vs Mach",
                             marker=dict(size=3, color=t, colorscale="Viridis",
                                         colorbar=dict(title="Time (s)")),
                             showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mass"], name="Mass",
                             line=dict(color="#8c564b", width=2)), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Mach", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Mach", row=1, col=1)
    fig.update_yaxes(title_text="G", row=1, col=2)
    fig.update_yaxes(title_text="α (°)", row=2, col=1)
    fig.update_yaxes(title_text="Mass (kg)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_forces(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Body Forces (3-axis)", "Body Accelerations (derived)",
                                        "Energy Budget", "Force Magnitude"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["force_x"], name="F_x (axial)",
                             line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_y"], name="F_y",
                             line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_z"], name="F_z",
                             line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["accel_x"], name="a_x",
                             line=dict(color="#1f77b4"), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["accel_y"], name="a_y",
                             line=dict(color="#2ca02c"), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["accel_z"], name="a_z",
                             line=dict(color="#d62728"), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["KE_kJ"], name="Kinetic",
                             line=dict(color="#ff7f0e"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["PE_kJ"], name="Potential",
                             line=dict(color="#2ca02c"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["total_energy_kJ"], name="Total",
                             line=dict(color="#1f77b4", width=2), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_total"], name="|F|",
                             line=dict(color="#9467bd"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Force (N)", row=1, col=1)
    fig.update_yaxes(title_text="Accel (m/s²)", row=1, col=2)
    fig.update_yaxes(title_text="Energy (kJ)", row=2, col=1)
    fig.update_yaxes(title_text="Force (N)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_control(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Fin Commands (PX4 → Bridge)",
                                        "Fin Actual (Actuator Model)",
                                        "Actuator Lag (Cmd − Actual)",
                                        "Command Envelope"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    for i in range(1, 5):
        fig.add_trace(go.Scatter(x=t, y=df[f"fin_cmd_{i}_deg"], name=f"Fin {i} cmd",
                                 line=dict(color=colors[i-1], width=1.5)), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=df[f"fin_act_{i}_deg"], name=f"Fin {i} act",
                                 line=dict(color=colors[i-1], width=1.5), showlegend=False), row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=df[f"fin_lag_{i}_deg"], name=f"Fin {i} lag",
                                 line=dict(color=colors[i-1], width=1), showlegend=False), row=2, col=1)
    all_cmd = np.concatenate([df[f"fin_cmd_{i}_deg"].values for i in range(1, 5)])
    all_t = np.tile(t.values, 4)
    fig.add_trace(go.Scatter(x=all_t, y=all_cmd, mode="markers",
                             marker=dict(size=2, color="#1f77b4", opacity=0.3),
                             name="All cmds", showlegend=False), row=2, col=2)
    fig.add_hline(y=20, line_dash="dash", line_color="red", opacity=0.5, row=2, col=2,
                  annotation_text="±20° limit")
    fig.add_hline(y=-20, line_dash="dash", line_color="red", opacity=0.5, row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Deflection (°)", row=1, col=1)
    fig.update_yaxes(title_text="Deflection (°)", row=1, col=2)
    fig.update_yaxes(title_text="Lag (°)", row=2, col=1)
    fig.update_yaxes(title_text="Deflection (°)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white",
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig


def _fig_3d_trajectory(df):
    PHASE_COLORS = {
        "BOOST": "#f44336", "COAST": "#2196f3", "TERMINAL": "#9c27b0",
        "ARMED": "#9e9e9e", "LAUNCH": "#ff9800", "CRUISE": "#4caf50",
        "BALLISTIC": "#795548",
    }
    has_pos = "pos_x" in df.columns and "pos_y" in df.columns
    fig = go.Figure()
    if has_pos and "flight_phase" in df.columns:
        from collections import OrderedDict
        for ph in OrderedDict.fromkeys(df["flight_phase"]):
            mask = df["flight_phase"] == ph
            c = PHASE_COLORS.get(ph, "#333")
            fig.add_trace(go.Scatter3d(
                x=df["pos_x"][mask], y=df["pos_y"][mask], z=df["alt_agl_m"][mask],
                mode="lines", name=ph, line=dict(color=c, width=5), legendgroup=ph,
                text=[f"t={t:.1f}s<br>Alt={a:.0f}m<br>V={v:.0f}m/s"
                      for t, a, v in zip(df["time_s"][mask], df["alt_agl_m"][mask],
                                         df["speed_total"][mask])],
                hoverinfo="text"
            ))
        fig.update_layout(
            scene=dict(xaxis_title="X North (m)", yaxis_title="Y East (m)",
                       zaxis_title="Altitude (m)", aspectmode="data"),
            height=650, template="plotly_white",
            title="3D Trajectory (colored by flight phase)",
            legend=dict(orientation="h", yanchor="bottom", y=1.02)
        )
    else:
        fig.add_trace(go.Scatter3d(
            x=df["ground_range"].values, y=np.zeros(len(df)), z=df["alt_agl_m"].values,
            mode="lines",
            line=dict(color=df["speed_total"].values, colorscale="Turbo", width=5,
                      colorbar=dict(title="Speed (m/s)", x=1.05)),
            text=[f"t={t:.1f}s<br>Alt={a:.0f}m<br>V={v:.0f}m/s<br>M={mach:.3f}"
                  for t, a, v, mach in zip(df["time_s"], df["alt_agl_m"],
                                           df["speed_total"], df["mach"])],
            hoverinfo="text"
        ))
        fig.update_layout(
            scene=dict(xaxis_title="Range (m)", yaxis_title="Cross-range (m)",
                       zaxis_title="Altitude AGL (m)", aspectmode="data"),
            height=600, template="plotly_white", title="3D Trajectory (colored by speed)"
        )
    return fig


def _fig_geo(df):
    if "lat" not in df.columns:
        return None
    fig = go.Figure(go.Scattermap(
        lat=df["lat"], lon=df["lon"],
        mode="lines+markers",
        marker=dict(size=4, color=df["time_s"], colorscale="Viridis",
                    colorbar=dict(title="Time (s)")),
        line=dict(width=2, color="#d62728"),
        text=[f"t={t:.1f}s Alt={a:.0f}m" for t, a in zip(df["time_s"], df["alt_msl"])],
        hoverinfo="text"
    ))
    center_lat = df["lat"].mean()
    center_lon = df["lon"].mean()
    fig.update_layout(
        map=dict(style="open-street-map",
                 center=dict(lat=center_lat, lon=center_lon), zoom=12),
        height=500, margin=dict(l=0, r=0, t=30, b=0), title="Ground Track"
    )
    return fig


def _fig_phase_portrait(df):
    t = df["time_s"]
    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Pitch Phase Portrait (θ vs q)",
                                        "α Phase Portrait (α vs q)"))
    fig.add_trace(go.Scatter(
        x=df["pitch_deg"], y=df["omega_y_deg"], mode="markers",
        marker=dict(size=3, color=t, colorscale="Viridis",
                    colorbar=dict(title="t (s)", x=0.45)),
        name="θ-q", showlegend=False), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=df["alpha_deg"], y=df["omega_y_deg"], mode="markers",
        marker=dict(size=3, color=t, colorscale="Plasma"),
        name="α-q", showlegend=False), row=1, col=2)
    fig.update_xaxes(title_text="Pitch (°)", row=1, col=1)
    fig.update_xaxes(title_text="α (°)", row=1, col=2)
    fig.update_yaxes(title_text="q (°/s)")
    fig.update_layout(height=450, template="plotly_white")
    return fig


def _fig_timing(timing_df):
    """MPC / MHE / cycle timing vs deadline."""
    if timing_df is None or len(timing_df) == 0:
        return None
    t = timing_df["t_sim"] if "t_sim" in timing_df.columns else np.arange(len(timing_df))
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("MPC solve time (µs)", "MHE solve time (µs)",
                                        "Full cycle time (µs)", "Distribution (MPC)"),
                        vertical_spacing=0.14, horizontal_spacing=0.08)
    deadline = TIMING_DEADLINE_US
    warn = TIMING_WARN_US

    if "mpc_us" in timing_df.columns:
        fig.add_trace(go.Scatter(x=t, y=timing_df["mpc_us"], name="mpc",
                                 line=dict(color="#e74c3c", width=1)), row=1, col=1)
        fig.add_hline(y=deadline, line_dash="dash", line_color="red", opacity=0.6, row=1, col=1,
                      annotation_text=f"deadline {deadline}µs")
        fig.add_hline(y=warn, line_dash="dot", line_color="orange", opacity=0.5, row=1, col=1,
                      annotation_text=f"warn {warn}µs")
        fig.add_trace(go.Histogram(x=timing_df["mpc_us"], nbinsx=50, name="mpc dist",
                                   marker_color="#e74c3c", showlegend=False), row=2, col=2)
    if "mhe_us" in timing_df.columns:
        fig.add_trace(go.Scatter(x=t, y=timing_df["mhe_us"], name="mhe",
                                 line=dict(color="#2ca02c", width=1), showlegend=False), row=1, col=2)
        fig.add_hline(y=deadline, line_dash="dash", line_color="red", opacity=0.4, row=1, col=2)
    if "cycle_us" in timing_df.columns:
        fig.add_trace(go.Scatter(x=t, y=timing_df["cycle_us"], name="cycle",
                                 line=dict(color="#1f77b4", width=1), showlegend=False), row=2, col=1)
        fig.add_hline(y=deadline, line_dash="dash", line_color="red", opacity=0.6, row=2, col=1,
                      annotation_text=f"deadline {deadline}µs")

    fig.update_xaxes(title_text="t_sim (s)", row=1, col=1)
    fig.update_xaxes(title_text="t_sim (s)", row=1, col=2)
    fig.update_xaxes(title_text="t_sim (s)", row=2, col=1)
    fig.update_xaxes(title_text="mpc_us", row=2, col=2)
    fig.update_yaxes(title_text="µs", row=1, col=1)
    fig.update_yaxes(title_text="µs", row=1, col=2)
    fig.update_yaxes(title_text="µs", row=2, col=1)
    fig.update_yaxes(title_text="count", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


# ─── HIL-specific figures ─────────────────────────────────────────────

def _fig_servo_hardware(df):
    """Cmd vs CAN feedback for each fin — core HIL validation plot."""
    if not all(f"fin_can_{i}_deg" in df.columns for i in range(1, 5)):
        return None
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=tuple(f"Fin {i} — Cmd vs CAN Feedback" for i in range(1, 5)),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    positions = [(1, 1), (1, 2), (2, 1), (2, 2)]
    colors_cmd = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    colors_fb = ["#aec7e8", "#ffbb78", "#98df8a", "#ff9896"]
    for i in range(1, 5):
        r, c = positions[i - 1]
        show = (i == 1)
        fig.add_trace(go.Scatter(
            x=t, y=df[f"fin_cmd_{i}_deg"], name="Cmd" if show else f"Cmd {i}",
            line=dict(color=colors_cmd[i-1], width=1.5),
            showlegend=show, legendgroup="cmd"), row=r, col=c)
        fig.add_trace(go.Scatter(
            x=t, y=df[f"fin_can_{i}_deg"], name="CAN FB" if show else f"CAN {i}",
            line=dict(color=colors_fb[i-1], width=1.5, dash="dot"),
            showlegend=show, legendgroup="fb"), row=r, col=c)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Deflection (°)")
    fig.update_layout(height=700, template="plotly_white",
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig


def _fig_servo_tracking_error(df):
    """Tracking error time series + histogram for each fin."""
    if not all(f"fin_can_err_{i}_deg" in df.columns for i in range(1, 5)):
        return None
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Tracking Error (all fins)", "Error Distribution",
                                        "Absolute Error per Fin", "Cumulative Error"),
                        vertical_spacing=0.14, horizontal_spacing=0.08)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    for i in range(1, 5):
        fig.add_trace(go.Scatter(
            x=t, y=df[f"fin_can_err_{i}_deg"], name=f"Fin {i}",
            line=dict(color=colors[i-1], width=1)), row=1, col=1)
    fig.add_hline(y=2.0, line_dash="dash", line_color="red", opacity=0.5, row=1, col=1,
                  annotation_text="+2°")
    fig.add_hline(y=-2.0, line_dash="dash", line_color="red", opacity=0.5, row=1, col=1,
                  annotation_text="-2°")
    all_err = np.concatenate([df[f"fin_can_err_{i}_deg"].values for i in range(1, 5)])
    fig.add_trace(go.Histogram(x=all_err, nbinsx=80, name="err dist",
                               marker_color="#9467bd", showlegend=False), row=1, col=2)
    for i in range(1, 5):
        fig.add_trace(go.Scatter(
            x=t, y=df[f"fin_can_err_{i}_deg"].abs(), name=f"|err| {i}",
            line=dict(color=colors[i-1], width=1), showlegend=False), row=2, col=1)
    fig.add_hline(y=2.0, line_dash="dash", line_color="red", opacity=0.5, row=2, col=1)
    for i in range(1, 5):
        cum_err = df[f"fin_can_err_{i}_deg"].abs().cumsum()
        fig.add_trace(go.Scatter(
            x=t, y=cum_err, name=f"cum {i}",
            line=dict(color=colors[i-1], width=1.5), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Error (°)", row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Cmd - CAN (°)", row=1, col=1)
    fig.update_yaxes(title_text="Count", row=1, col=2)
    fig.update_yaxes(title_text="|Error| (°)", row=2, col=1)
    fig.update_yaxes(title_text="Cumulative |err| (°)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_can_health(df, servo_df=None):
    """CAN health: fin_source timeline + pie, online_mask, tx_fail."""
    has_source = "fin_source" in df.columns
    has_servo = servo_df is not None and len(servo_df) > 0
    if not has_source and not has_servo:
        return None

    n_rows = 2 if has_servo and "online_mask" in (servo_df.columns if has_servo else []) else 1
    specs = [[{"type": "xy"}, {"type": "domain"}]]
    titles = ["fin_source Timeline", "Fin Source Breakdown"]
    if n_rows == 2:
        specs.append([{"type": "xy", "colspan": 2}, None])
        titles.append("Online Mask & TX Fail")

    fig = make_subplots(rows=n_rows, cols=2, specs=specs,
                        subplot_titles=titles,
                        vertical_spacing=0.16, horizontal_spacing=0.08)

    if has_source:
        source_map = {"can": 3, "hold": 2, "cmd": 1, "abort": 0}
        src_num = df["fin_source"].map(source_map).fillna(-1)
        fig.add_trace(go.Scatter(
            x=df["time_s"], y=src_num, mode="markers",
            marker=dict(size=3, color=src_num,
                        colorscale=[[0, "#e53935"], [0.33, "#2196f3"],
                                    [0.66, "#ff9800"], [1.0, "#4caf50"]]),
            name="source", showlegend=False), row=1, col=1)
        fig.update_yaxes(tickvals=[0, 1, 2, 3],
                         ticktext=["abort", "cmd", "hold", "can"], row=1, col=1)
        fig.update_xaxes(title_text="Time (s)", row=1, col=1)

        src_counts = df["fin_source"].value_counts()
        color_map = {"can": "#4caf50", "hold": "#ff9800", "cmd": "#2196f3", "abort": "#e53935"}
        labels = src_counts.index.tolist()
        pie_colors = [color_map.get(l, "#999") for l in labels]
        fig.add_trace(go.Pie(labels=labels, values=src_counts.values.tolist(),
                             marker=dict(colors=pie_colors),
                             textinfo="label+percent", hole=0.4,
                             name="fin_source"), row=1, col=2)

    if n_rows == 2 and has_servo and "online_mask" in servo_df.columns:
        t_srv = servo_df["t_sim"] if "t_sim" in servo_df.columns else np.arange(len(servo_df))
        masks = servo_df["online_mask"].values.astype(int)
        n_online = np.array([bin(m & 0x0F).count('1') for m in masks])
        fig.add_trace(go.Scatter(
            x=t_srv, y=n_online, name="servos online",
            line=dict(color="#4caf50", width=2),
            fill="tozeroy", fillcolor="rgba(76,175,80,0.15)"), row=2, col=1)
        fig.update_yaxes(title_text="# Servos Online", range=[-0.2, 4.5], row=2, col=1)
        if "tx_fail" in servo_df.columns:
            tx = servo_df["tx_fail"].values.astype(int)
            tx_delta = np.diff(tx, prepend=tx[0])
            tx_delta = np.clip(tx_delta, 0, None)
            fig.add_trace(go.Bar(
                x=t_srv, y=tx_delta, name="tx_fail delta",
                marker_color="rgba(229,57,53,0.6)", showlegend=False), row=2, col=1)
        fig.update_xaxes(title_text="Time (s)", row=2, col=1)

    fig.update_layout(height=600 if n_rows == 1 else 800, template="plotly_white")
    return fig


# ══════════════════════════════════════════════════════════════════════
#  HTML Generation
# ══════════════════════════════════════════════════════════════════════

_CSS = """\
:root{--bg:#f8f9fa;--card:#fff;--border:#e0e0e0;--text:#222;--text-secondary:#666;
--text-muted:#999;--accent:#1565c0;--pass:#4caf50;--warn:#ff9800;--fail:#e53935;
--th-bg:#f0f4f8;--hover:rgba(21,101,192,.04);--diag-error-bg:#fbe9e7;
--diag-warn-bg:#fff3e0;--diag-info-bg:#e3f2fd;--rec-bg:#e8f5e9}
[data-theme="dark"]{--bg:#121212;--card:#1e1e1e;--border:#333;--text:#e0e0e0;
--text-secondary:#aaa;--text-muted:#777;--accent:#42a5f5;--th-bg:#2a2a2a;
--hover:rgba(66,165,245,.08);--diag-error-bg:#3e1a1a;--diag-warn-bg:#3e2e1a;
--diag-info-bg:#1a2e3e;--rec-bg:#1a3e1a}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,-apple-system,sans-serif;background:var(--bg);
color:var(--text);line-height:1.6;padding:20px;transition:background .3s,color .3s}
.container{max-width:1400px;margin:0 auto}
h1{font-size:1.8rem;border-bottom:3px solid var(--accent);padding-bottom:8px;margin-bottom:16px}
h2{font-size:1.3rem;color:var(--accent);margin:24px 0 12px;border-left:4px solid var(--accent);padding-left:10px}
.grid{display:grid;gap:16px}.grid-2{grid-template-columns:1fr 1fr}
.grid-3{grid-template-columns:1fr 1fr 1fr}.grid-4{grid-template-columns:repeat(4,1fr)}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;
padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.08);transition:background .3s,border-color .3s}
.score-ring{width:120px;height:120px;border-radius:50%;display:flex;align-items:center;
justify-content:center;font-size:2rem;font-weight:700;margin:0 auto 8px;border:6px solid}
.score-ring.pass{border-color:var(--pass);color:var(--pass)}
.score-ring.warn{border-color:var(--warn);color:var(--warn)}
.score-ring.fail{border-color:var(--fail);color:var(--fail)}
.badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.75rem;
font-weight:700;color:#fff;text-transform:uppercase}
.badge.pass{background:var(--pass)}.badge.warn{background:var(--warn)}.badge.fail{background:var(--fail)}
.badge.info{background:#2196f3}
.metric-box{text-align:center;padding:12px}
.metric-box .value{font-size:1.6rem;font-weight:700;color:var(--accent)}
.metric-box .label{font-size:.75rem;color:var(--text-secondary);text-transform:uppercase;letter-spacing:.5px}
.metric-box .sub{font-size:.7rem;color:var(--text-muted)}
table{width:100%;border-collapse:collapse;font-size:.85rem}
th{background:var(--th-bg);padding:8px 12px;text-align:left;border-bottom:2px solid var(--border);font-weight:600}
td{padding:6px 12px;border-bottom:1px solid var(--border)}tr:hover{background:var(--hover)}
.diag{padding:10px 14px;border-radius:6px;margin-bottom:8px;border-left:4px solid}
.diag.error{background:var(--diag-error-bg);border-color:var(--fail)}
.diag.warning{background:var(--diag-warn-bg);border-color:var(--warn)}
.diag.info{background:var(--diag-info-bg);border-color:#2196f3}
.diag .dtitle{font-weight:700;font-size:.9rem}
.diag .ddetail{font-size:.8rem;color:var(--text-secondary);margin-top:2px}
.rec{padding:8px 14px;background:var(--rec-bg);border-radius:6px;margin-bottom:6px;
font-size:.85rem;border-left:3px solid var(--pass)}
.tabs{display:flex;gap:4px;border-bottom:2px solid var(--border);flex-wrap:wrap}
.tab-btn{padding:8px 20px;border:none;background:none;cursor:pointer;font-size:.9rem;
font-weight:600;border-bottom:3px solid transparent;color:var(--text-secondary);transition:.2s}
.tab-btn:hover{color:var(--accent)}.tab-btn.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-panel{display:none;padding:16px 0}.tab-panel.active{display:block}
.toolbar{display:flex;align-items:center;gap:12px;margin-bottom:16px;flex-wrap:wrap}
.theme-toggle{background:var(--card);border:1px solid var(--border);border-radius:20px;
padding:6px 14px;cursor:pointer;font-size:.85rem;color:var(--text);transition:.2s;
display:flex;align-items:center;gap:6px}.theme-toggle:hover{border-color:var(--accent)}
.hil-banner{background:linear-gradient(90deg,#1565c0,#1976d2);color:#fff;padding:6px 14px;
border-radius:20px;font-size:.8rem;font-weight:700}
@media(max-width:900px){.grid-2,.grid-3,.grid-4{grid-template-columns:1fr}}
"""

_JS = """\
function openTab(evt,tabId){
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  evt.currentTarget.classList.add('active');
  var el=document.getElementById(tabId);
  el.querySelectorAll('.js-plotly-plot').forEach(function(p){Plotly.Plots.resize(p);});
}
(function(){
  var saved=localStorage.getItem('m130_hil_theme')||'light';
  document.documentElement.setAttribute('data-theme',saved);
  window.addEventListener('DOMContentLoaded',function(){
    var btn=document.getElementById('theme-toggle');
    if(!btn)return;
    _updateToggleLabel(btn,saved);
    btn.addEventListener('click',function(){
      var cur=document.documentElement.getAttribute('data-theme')||'light';
      var next=cur==='dark'?'light':'dark';
      document.documentElement.setAttribute('data-theme',next);
      localStorage.setItem('m130_hil_theme',next);
      _updateToggleLabel(btn,next);
      var bg=next==='dark'?'#1e1e1e':'#fff';
      var fg=next==='dark'?'#e0e0e0':'#444';
      document.querySelectorAll('.js-plotly-plot').forEach(function(p){
        Plotly.relayout(p,{'paper_bgcolor':bg,'plot_bgcolor':bg,'font.color':fg});
      });
    });
  });
  function _updateToggleLabel(btn,theme){
    btn.textContent=theme==='dark'?'\\u2600 Light Mode':'\\u263e Dark Mode';
  }
})();
"""

plotly_cdn = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


def _metric_card(label, value, sub="", color="var(--accent)"):
    return (f'<div class="metric-box"><div class="value" style="color:{color}">{value}</div>'
            f'<div class="label">{label}</div><div class="sub">{sub}</div></div>')


def _plotly_div(fig):
    if fig is None:
        return ""
    config = {"responsive": True, "displayModeBar": True,
              "modeBarButtonsToRemove": ["lasso2d", "select2d"]}
    return pio.to_html(fig, full_html=False, include_plotlyjs=False, config=config)


def generate_html(df, metrics, scores, diags, recs,
                  timing_df=None, servo_df=None, html_path=None):
    m = metrics
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    overall = scores["overall"]
    total = scores["total"]
    ring_cls = overall.lower()
    score_html = (
        f'<div class="card" style="text-align:center">'
        f'<div class="score-ring {ring_cls}">{total}</div>'
        f'<div style="font-weight:700;font-size:1.1rem">{overall}</div>'
        f'<div style="font-size:.8rem;color:var(--text-secondary)">/ 100</div></div>'
    )

    checks_rows = ""
    for name, verdict, detail in scores["checks"]:
        badge = f'<span class="badge {verdict.lower()}">{verdict}</span>'
        checks_rows += f'<tr><td>{name}</td><td>{badge}</td><td>{detail}</td></tr>'
    checks_html = (
        f'<div class="card"><h3 style="margin-bottom:8px">Health Checks</h3>'
        f'<table><tr><th>Check</th><th>Status</th><th>Detail</th></tr>'
        f'{checks_rows}</table></div>'
    )

    # Timing mini-card if available
    timing_card = ""
    if m.get("timing_samples", 0) > 0:
        mpc_avg = m.get("mpc_us_avg", 0) / 1000
        mpc_max = m.get("mpc_us_max", 0) / 1000
        over_pct = m.get("mpc_over_deadline_pct", 0)
        color = "var(--pass)" if over_pct < 10 else ("var(--warn)" if over_pct < 50 else "var(--fail)")
        timing_card = (
            f'<div class="card">'
            f'<h3 style="margin-bottom:8px">MPC Timing (ARM64)</h3>'
            f'<table>'
            f'<tr><td>Samples</td><td>{m["timing_samples"]}</td></tr>'
            f'<tr><td>MPC avg</td><td>{mpc_avg:.1f} ms</td></tr>'
            f'<tr><td>MPC max</td><td>{mpc_max:.1f} ms</td></tr>'
            f'<tr><td>Over deadline</td><td style="color:{color};font-weight:700">'
            f'{over_pct:.1f}%</td></tr>'
            f'</table></div>'
        )

    # Servo mini-card (HIL-specific)
    servo_card = ""
    mae = m.get("servo_tracking_mae_deg", 0)
    can_pct = m.get("fin_source_can_pct", 0)
    online_pct = m.get("servo_all_online_pct", 0)
    if mae > 0 or m.get("servo_samples", 0) > 0:
        mae_color = "var(--pass)" if mae < SERVO_MAE_MAX_DEG else (
            "var(--warn)" if mae < SERVO_MAE_MAX_DEG * 2 else "var(--fail)")
        servo_card = (
            f'<div class="card">'
            f'<h3 style="margin-bottom:8px">Servo Hardware (CAN)</h3>'
            f'<table>'
            f'<tr><td>Tracking MAE</td><td style="color:{mae_color};font-weight:700">'
            f'{mae:.2f}\u00b0</td></tr>'
            f'<tr><td>Tracking P95</td><td>{m.get("servo_tracking_p95_deg", 0):.2f}\u00b0</td></tr>'
            f'<tr><td>CAN source</td><td>{can_pct:.0f}%</td></tr>'
            f'<tr><td>All online</td><td>{online_pct:.0f}%</td></tr>'
            f'<tr><td>TX failures</td><td>{m.get("servo_tx_fail_total", 0)}</td></tr>'
            f'</table></div>'
        )

    key_cards = f"""
    <div class="grid grid-4" style="margin-bottom:16px">
      <div class="card">{_metric_card("Range", f"{m['impact_range_m']:.0f}m",
                                       f"err {m['range_error_pct']:+.1f}%",
                                       "var(--pass)" if abs(m['range_error_pct'])<5 else "var(--warn)")}</div>
      <div class="card">{_metric_card("Peak Alt AGL", f"{m['peak_alt_m']:.0f}m",
                                       f"at t={m['peak_alt_time_s']:.1f}s")}</div>
      <div class="card">{_metric_card("Max Mach", f"{m['max_mach']:.3f}",
                                       f"Speed {m['max_speed_mps']:.0f} m/s")}</div>
      <div class="card">{_metric_card("Flight Time", f"{m['flight_time_s']:.1f}s",
                                       f"{m['n_steps']} steps")}</div>
    </div>
    <div class="grid grid-4" style="margin-bottom:16px">
      <div class="card">{_metric_card("Max |α| (flight)",
                                       f"{m.get('max_alpha_flight_deg', m['max_alpha_deg']):.1f}°",
                                       "excludes terminal tumble")}</div>
      <div class="card">{_metric_card("Max G", f"{m['max_g']:.1f}g",
                                       f"Axial: {m['max_axial_g']:.1f}g")}</div>
      <div class="card">{_metric_card("Servo MAE", f"{mae:.2f}°",
                                       f"CAN: {can_pct:.0f}% | Online: {online_pct:.0f}%")}</div>
      <div class="card">{_metric_card("Propellant", f"{m['propellant_kg']:.2f} kg",
                                       f"{m['mass_initial']:.2f} → {m['mass_final']:.2f} kg")}</div>
    </div>
    """

    diag_html = ""
    for level, title, detail in diags:
        diag_html += (f'<div class="diag {level}"><div class="dtitle">{title}</div>'
                      f'<div class="ddetail">{detail}</div></div>')
    if not diag_html:
        diag_html = ('<div class="diag info"><div class="dtitle">All Clear</div>'
                     '<div class="ddetail">No issues detected</div></div>')

    rec_html = "".join(f'<div class="rec">{r}</div>' for r in recs)

    # Figures
    fig_traj = _fig_trajectory(df)
    fig_att = _fig_attitude(df)
    fig_aero = _fig_aero(df)
    fig_forces = _fig_forces(df)
    fig_ctrl = _fig_control(df)
    fig_3d = _fig_3d_trajectory(df)
    fig_geo = _fig_geo(df)
    fig_portrait = _fig_phase_portrait(df)
    fig_timing = _fig_timing(timing_df)
    fig_servo_hw = _fig_servo_hardware(df)
    fig_servo_err = _fig_servo_tracking_error(df)
    fig_can = _fig_can_health(df, servo_df)

    tabs = [
        ("Overview", "tab-overview"),
        ("Trajectory", "tab-traj"),
        ("3D View", "tab-3d"),
        ("Attitude", "tab-att"),
        ("Aero & Mass", "tab-aero"),
        ("Forces", "tab-forces"),
        ("Control", "tab-ctrl"),
    ]
    if fig_servo_hw:
        tabs.append(("Servo Hardware", "tab-servo"))
    if fig_servo_err:
        tabs.append(("Tracking Error", "tab-trkerr"))
    if fig_can:
        tabs.append(("CAN Health", "tab-can"))
    tabs.append(("Phase Portrait", "tab-portrait"))
    if fig_timing:
        tabs.append(("Timing (ARM64)", "tab-timing"))
    if fig_geo:
        tabs.append(("Ground Track", "tab-geo"))

    tabs_header = '<div class="tabs">'
    for i, (label, tid) in enumerate(tabs):
        active = " active" if i == 0 else ""
        tabs_header += f'<button class="tab-btn{active}" onclick="openTab(event,\'{tid}\')">{label}</button>'
    tabs_header += '</div>'

    overview_grid = (
        f'<div class="grid grid-3" style="margin-bottom:16px">'
        f'{score_html}{checks_html}'
        f'<div class="card"><h3 style="margin-bottom:8px">Diagnostics</h3>{diag_html}</div>'
        f'</div>'
    )
    side_cards = timing_card + servo_card
    if side_cards:
        overview_grid += f'<div class="grid grid-2" style="margin-bottom:16px">{side_cards}</div>'

    tab_overview = f"""
    <div id="tab-overview" class="tab-panel active">
      <h2>Mission Summary</h2>
      {overview_grid}
      <div class="card" style="margin-bottom:16px"><h3 style="margin-bottom:8px">Recommendations</h3>{rec_html}</div>
      <h2>Flight Metrics</h2>
      {key_cards}
    </div>"""

    tab_traj = f'<div id="tab-traj" class="tab-panel"><h2>Trajectory</h2>{_plotly_div(fig_traj)}</div>'
    tab_3d = f'<div id="tab-3d" class="tab-panel"><h2>3D Trajectory</h2>{_plotly_div(fig_3d)}</div>'
    tab_att = f'<div id="tab-att" class="tab-panel"><h2>Attitude & Angular Rates</h2>{_plotly_div(fig_att)}</div>'
    tab_aero = f'<div id="tab-aero" class="tab-panel"><h2>Aerodynamics & Mass</h2>{_plotly_div(fig_aero)}</div>'
    tab_forces = f'<div id="tab-forces" class="tab-panel"><h2>Forces & Energy</h2>{_plotly_div(fig_forces)}</div>'
    tab_ctrl = f'<div id="tab-ctrl" class="tab-panel"><h2>Control & Actuators (PX4 HIL)</h2>{_plotly_div(fig_ctrl)}</div>'
    tab_servo = ""
    if fig_servo_hw:
        tab_servo = f'<div id="tab-servo" class="tab-panel"><h2>Servo Hardware — Cmd vs CAN Feedback</h2>{_plotly_div(fig_servo_hw)}</div>'
    tab_trkerr = ""
    if fig_servo_err:
        tab_trkerr = f'<div id="tab-trkerr" class="tab-panel"><h2>Servo Tracking Error Analysis</h2>{_plotly_div(fig_servo_err)}</div>'
    tab_can = ""
    if fig_can:
        tab_can = f'<div id="tab-can" class="tab-panel"><h2>CAN Bus Health</h2>{_plotly_div(fig_can)}</div>'
    tab_portrait = f'<div id="tab-portrait" class="tab-panel"><h2>Phase Portraits</h2>{_plotly_div(fig_portrait)}</div>'
    tab_timing = ""
    if fig_timing:
        tab_timing = f'<div id="tab-timing" class="tab-panel"><h2>ARM64 Realtime Timing</h2>{_plotly_div(fig_timing)}</div>'
    tab_geo = ""
    if fig_geo:
        tab_geo = f'<div id="tab-geo" class="tab-panel"><h2>Ground Track (Map)</h2>{_plotly_div(fig_geo)}</div>'

    toolbar = (
        '<div class="toolbar">'
        '<button id="theme-toggle" class="theme-toggle">&#9790; Dark Mode</button>'
        '<span class="hil-banner">HIL — Hardware-In-the-Loop (ARM64 + CAN Servos)</span>'
        '<span style="font-size:.8rem;color:var(--text-secondary)">PX4 + real servos on real hardware</span>'
        '</div>'
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M130 HIL Analysis &mdash; {html_escape(m['timestamp'])}</title>
{plotly_cdn}
<style>{_CSS}</style>
</head>
<body>
<div class="container">
  <h1>M130 HIL Flight Analysis</h1>
  {toolbar}
  <div style="color:var(--text-secondary);font-size:.85rem;margin-bottom:16px">
    File: {html_escape(m['file'])} | Generated: {now}
  </div>
  {key_cards}
  <div class="tab-container">
    {tabs_header}
    {tab_overview}
    {tab_traj}
    {tab_3d}
    {tab_att}
    {tab_aero}
    {tab_forces}
    {tab_ctrl}
    {tab_servo}
    {tab_trkerr}
    {tab_can}
    {tab_portrait}
    {tab_timing}
    {tab_geo}
  </div>
</div>
<script>{_JS}</script>
</body>
</html>"""

    if html_path:
        Path(html_path).write_text(html, encoding="utf-8")
    return html


# ══════════════════════════════════════════════════════════════════════
#  Console Summary
# ══════════════════════════════════════════════════════════════════════

def print_console_summary(m, scores):
    tag = {"PASS": "✅", "WARN": "⚠️", "FAIL": "❌"}[scores["overall"]]
    print()
    print("═" * 62)
    print(f"  M130 HIL Analysis (ARM64 + CAN)  {tag} {scores['overall']} ({scores['total']}/100)")
    print("═" * 62)
    print(f"  Range:     {m['impact_range_m']:.0f}m  (err {m['range_error_pct']:+.1f}%)")
    print(f"  Peak Alt:  {m['peak_alt_m']:.0f}m AGL   Max Mach: {m['max_mach']:.3f}   Max G: {m['max_g']:.1f}")
    print(f"  Time:      {m['flight_time_s']:.2f}s   "
          f"Max |α|(flight): {m.get('max_alpha_flight_deg', m['max_alpha_deg']):.1f}°   "
          f"Max fin: {m['max_fin_cmd_deg']:.1f}°")
    print(f"  Mass:      {m['mass_initial']:.2f} → {m['mass_final']:.2f} kg   "
          f"Fin saturation: {m['fin_saturation_pct']:.1f}%")
    # Servo hardware
    mae = m.get("servo_tracking_mae_deg", 0)
    can_pct = m.get("fin_source_can_pct", 0)
    if mae > 0 or m.get("servo_samples", 0) > 0:
        print(f"  Servo:     MAE={mae:.2f}°  P95={m.get('servo_tracking_p95_deg',0):.2f}°  "
              f"CAN={can_pct:.0f}%  online={m.get('servo_all_online_pct',0):.0f}%  "
              f"tx_fail={m.get('servo_tx_fail_total',0)}")
    else:
        print(f"  Servo:     (no servo feedback data)")
    if m.get("timing_samples", 0) > 0:
        print(f"  MPC timing: avg={m.get('mpc_us_avg',0)/1000:.1f}ms  max={m.get('mpc_us_max',0)/1000:.1f}ms  "
              f"over-deadline: {m.get('mpc_over_deadline_pct',0):.0f}%")
    else:
        print(f"  MPC timing: (no timing samples — check adb reverse tcp:5760)")
    print("═" * 62)


# ══════════════════════════════════════════════════════════════════════
#  Entry Points
# ══════════════════════════════════════════════════════════════════════

def analyze_hil_csv(csv_path, open_browser=True):
    csv_path = Path(csv_path)
    df = load_hil_csv(csv_path)
    if len(df) == 0:
        print(f"  ERROR: empty CSV — {csv_path}")
        return {}, {"overall": "FAIL", "total": 0, "checks": []}, ""
    timing_df = load_hil_timing(csv_path)
    servo_df = load_hil_servos(csv_path)
    metrics = extract_metrics(df, csv_path, timing_df=timing_df, servo_df=servo_df)
    scores = score_run(metrics)
    diags = diagnose(df, metrics)
    recs = recommend(metrics, scores, diags)

    out_dir = csv_path.parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    html_path = out_dir / f"hil_analysis_{metrics['timestamp']}.html"

    generate_html(df, metrics, scores, diags, recs,
                  timing_df=timing_df, servo_df=servo_df,
                  html_path=str(html_path))
    print_console_summary(metrics, scores)
    print(f"  HTML → {html_path}")

    if open_browser:
        webbrowser.open(f"file://{html_path.resolve()}")

    return metrics, scores, str(html_path)


def discover_hil_csvs(results_dir: Path) -> list:
    return sorted(results_dir.glob("hil_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)


def main():
    parser = argparse.ArgumentParser(description="M130 HIL Flight — Interactive HTML Analysis")
    parser.add_argument("--file", type=str, default=None, help="Analyze specific CSV")
    parser.add_argument("--no-open", action="store_true", help="Don't open browser")
    args = parser.parse_args()

    if args.file:
        csv_files = [Path(args.file)]
    else:
        csv_files = discover_hil_csvs(_RESULTS_DIR)
        # exclude timing and servo CSVs
        csv_files = [p for p in csv_files if "timing" not in p.name and "servo" not in p.name]

    if not csv_files:
        print("ERROR: no hil_*.csv files found in", _RESULTS_DIR)
        sys.exit(1)

    print(f"  Analyzing: {csv_files[0].name}")
    analyze_hil_csv(csv_files[0], open_browser=not args.no_open)


if __name__ == "__main__":
    main()
