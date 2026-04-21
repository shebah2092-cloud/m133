#!/usr/bin/env python3
"""
PIL Test Runner — مستقل تماماً
==============================

المكوّنات:
  1) Python baseline (rocket_6dof_sim + MPC داخلي)  ← مرجع
  2) PIL bridge (mavlink_bridge_pil)                ← يتصل بجهاز ARM64 خارجي

المقارنة الوحيدة: baseline ↔ PIL (لا علاقة بـ SITL).

Usage:
    python pil_runner.py                        # الكامل: baseline → PIL → مقارنة
    python pil_runner.py --pil-only             # تشغيل PIL فقط
    python pil_runner.py --baseline-only        # تشغيل baseline فقط
    python pil_runner.py --compare-only         # مقارنة من CSV موجودة
    python pil_runner.py --baseline-csv X.csv   # تحديد baseline مسبقاً
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS = _SCRIPT_DIR / "results"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SCRIPT_DIR))


# ============================================================================
# baseline (مستقل عن sitl/run_sitl_test.py)
# ============================================================================

def run_baseline(sim_config_path: str | None = None,
                 output_csv: str | None = None) -> str:
    """يشغّل المحاكاة المستقلة (Python + MPC الداخلي) كمرجع ذهبي."""
    import tempfile

    from rocket_6dof_sim import Rocket6DOFSimulation, export_comprehensive_log

    cfg_path = sim_config_path or str(_SIM_DIR / "config" / "6dof_config_advanced.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["simulation"]["control_type"] = "mpc"

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml",
                                      delete=False, encoding="utf-8")
    yaml.dump(cfg, tmp, allow_unicode=True, default_flow_style=False)
    tmp.close()

    long_range = cfg.get("long_range", {}).get("enabled", False)
    sim = Rocket6DOFSimulation(config_file=tmp.name, long_range_mode=long_range)

    duration = cfg.get("simulation", {}).get("duration", 500.0)
    dt = cfg.get("simulation", {}).get("dt", 0.01)

    t0 = time.monotonic()

    def prog(info):
        print(f"\r  [baseline] {info['progress']:5.1f}% t={info['time']:6.1f}s "
              f"alt={info['altitude']/1000:.1f}km v={info['velocity']:.0f}m/s",
              end="", flush=True)

    hist = sim.simulate(duration=duration, dt=dt, on_step=prog, callback_stride=100)
    print()
    print(f"  baseline wall={time.monotonic() - t0:.1f}s "
          f"points={len(hist['time'])}")

    csv_path = output_csv or str(_RESULTS / "baseline_flight.csv")
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    export_comprehensive_log(hist, sim, csv_path)
    os.unlink(tmp.name)
    print(f"  baseline CSV: {csv_path}")
    return csv_path


# ============================================================================
# PIL
# ============================================================================

def run_pil(cfg_path: str, flight_csv: str, timing_csv: str) -> tuple[str, str]:
    """يُطلق جسر PIL المستقل وينتظر اتصال الهدف."""
    from mavlink_bridge_pil import PILBridge

    print("=" * 70)
    print("  PIL: connecting to remote target")
    print("=" * 70)

    bridge = PILBridge(cfg_path)
    bridge.run(flight_csv, timing_csv)
    return flight_csv, timing_csv


# ============================================================================
# تحميل CSV + مواءمة الأعمدة
# ============================================================================

_ALIASES = {
    "time":         ["time", "time_s"],
    "vel_x":        ["vel_x", "velocity_x_m_s"],
    "vel_y":        ["vel_y", "velocity_y_m_s"],
    "vel_z":        ["vel_z", "velocity_z_m_s"],
    "pos_x":        ["pos_x", "position_x_m"],
    "pos_y":        ["pos_y", "position_y_m"],
    "pos_z":        ["pos_z", "position_z_m"],
    "altitude":     ["altitude", "altitude_m"],
    "altitude_msl": ["altitude_msl", "altitude_lla_m", "altitude_m"],
    "ground_range": ["ground_range", "ground_range_m"],
    "q0": ["q0", "quat_w"], "q1": ["q1", "quat_x"],
    "q2": ["q2", "quat_y"], "q3": ["q3", "quat_z"],
    "fin_cmd_1": ["fin_cmd_1", "fin_1_rad"],
    "fin_cmd_2": ["fin_cmd_2", "fin_2_rad"],
    "fin_cmd_3": ["fin_cmd_3", "fin_3_rad"],
    "fin_cmd_4": ["fin_cmd_4", "fin_4_rad"],
    "omega_x": ["omega_x", "omega_x_rad_s"],
    "omega_y": ["omega_y", "omega_y_rad_s"],
    "omega_z": ["omega_z", "omega_z_rad_s"],
    "mass": ["mass", "mass_kg"],
}


def _q2euler_deg(q0, q1, q2, q3):
    r = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    p = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1.0, 1.0))
    y = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return np.degrees(r), np.degrees(p), np.degrees(y)


def load_csv(path: str) -> dict:
    raw: dict[str, list] = {}
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for k in r.fieldnames or []:
            raw[k] = []
        for row in r:
            for k in r.fieldnames or []:
                try:
                    raw[k].append(float(row[k]))
                except (ValueError, TypeError):
                    raw[k].append(0.0)
    data = {k: np.array(v) for k, v in raw.items()}
    for canon, cands in _ALIASES.items():
        if canon in data:
            continue
        for c in cands:
            if c in data:
                data[canon] = data[c]
                break
    if all(k in data for k in ("q0", "q1", "q2", "q3")) and "pitch_deg" not in data:
        r, p, y = _q2euler_deg(data["q0"], data["q1"], data["q2"], data["q3"])
        data["roll_deg"] = r
        data["pitch_deg"] = p
        data["yaw_deg"] = y
    if all(k in data for k in ("vel_x", "vel_y", "vel_z")) and "vel_total" not in data:
        data["vel_total"] = np.sqrt(data["vel_x"] ** 2
                                    + data["vel_y"] ** 2
                                    + data["vel_z"] ** 2)
    return data


# ============================================================================
# المقارنة: baseline ↔ PIL فقط
# ============================================================================

def compare(baseline_csv: str, pil_csv: str, timing_csv: str | None,
            thresholds: dict) -> dict:
    print()
    print("=" * 70)
    print("  Compare: baseline ↔ PIL")
    print("=" * 70)

    base = load_csv(baseline_csv)
    pil = load_csv(pil_csv)

    t_b = base.get("time", np.array([]))
    t_p = pil.get("time", np.array([]))
    if len(t_b) == 0 or len(t_p) == 0:
        print("  ERROR: empty data")
        return {"pass": False, "error": "empty"}

    t_max = min(t_b[-1], t_p[-1])
    mask = t_b <= t_max
    t_common = t_b[mask]

    def ib(key):
        return base[key][mask] if key in base else np.zeros_like(t_common)

    def ip(key):
        return np.interp(t_common, t_p, pil[key]) if key in pil else np.zeros_like(t_common)

    m: dict = {"t_final_base": float(t_b[-1]), "t_final_pil": float(t_p[-1])}

    # ارتفاع
    a_err = ip("altitude") - ib("altitude")
    m["altitude_mae"] = float(np.mean(np.abs(a_err)))
    m["altitude_max_err"] = float(np.max(np.abs(a_err)))
    m["altitude_rmse"] = float(np.sqrt(np.mean(a_err ** 2)))

    # سرعة (MAE للمتّجه)
    if all(k in base and k in pil for k in ("vel_x", "vel_y", "vel_z")):
        dvx = ip("vel_x") - ib("vel_x")
        dvy = ip("vel_y") - ib("vel_y")
        dvz = ip("vel_z") - ib("vel_z")
        v_err = np.sqrt(dvx ** 2 + dvy ** 2 + dvz ** 2)
        m["velocity_mae"] = float(np.mean(v_err))
        m["velocity_max_err"] = float(np.max(v_err))

    # زوايا
    if "pitch_deg" in base and "pitch_deg" in pil:
        def _ipd(k): return np.interp(t_common, t_p, pil[k])
        def _ibd(k): return base[k][mask]
        pitch_err = _ipd("pitch_deg") - _ibd("pitch_deg")
        yaw_err = _ipd("yaw_deg") - _ibd("yaw_deg")
        roll_err = _ipd("roll_deg") - _ibd("roll_deg")
        m["attitude_max_deg"] = float(max(
            np.max(np.abs(pitch_err)),
            np.max(np.abs(yaw_err)),
            np.max(np.abs(roll_err)),
        ))

    # CEP عند الاصطدام
    if all(k in base and k in pil for k in ("pos_x", "pos_y")):
        dx = float(pil["pos_x"][-1] - base["pos_x"][-1])
        dy = float(pil["pos_y"][-1] - base["pos_y"][-1])
        m["cep_m"] = float(np.sqrt(dx * dx + dy * dy))

    # نطاق
    if "ground_range" in base and "ground_range" in pil:
        r_err = ip("ground_range") - ib("ground_range")
        m["range_mae"] = float(np.mean(np.abs(r_err)))

    # التوقيت
    timing = _analyze_timing(timing_csv) if timing_csv and Path(timing_csv).exists() else {}
    m["timing"] = timing

    # بوابة
    m["pass"] = _gate(m, thresholds)

    _print_summary(m, thresholds)
    return m


def _analyze_timing(path: str) -> dict:
    cyc, mpc, mhe = [], [], []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                cyc.append(float(row.get("cycle_us", 0)))
                mpc.append(float(row.get("mpc_us", 0)))
                mhe.append(float(row.get("mhe_us", 0)))
            except (ValueError, TypeError):
                continue

    def stats(a):
        a = np.asarray(a, dtype=float)
        if a.size == 0:
            return {}
        return {
            "min": float(a.min()), "mean": float(a.mean()),
            "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)),
            "max": float(a.max()),
        }

    return {
        "samples": len(cyc),
        "mhe_us": stats(mhe),
        "mpc_us": stats(mpc),
        "cycle_us": stats(cyc),
        "deadline_miss": int(sum(1 for c in cyc if c > 20000)),
    }


def _gate(m: dict, th: dict) -> bool:
    ok = True
    vb = th.get("vs_baseline", {})
    tg = th.get("timing", {})

    if m.get("velocity_mae", 0) > vb.get("velocity_mae_max", 1e9):
        ok = False
    if m.get("attitude_max_deg", 0) > vb.get("attitude_max_deg", 1e9):
        ok = False
    if m.get("altitude_mae", 0) > vb.get("altitude_mae_max", 1e9):
        ok = False
    if m.get("cep_m", 0) > vb.get("cep_max_m", 1e9):
        ok = False

    t = m.get("timing", {})
    if t:
        mpc_p95 = t.get("mpc_us", {}).get("p95", 0)
        cyc_p95 = t.get("cycle_us", {}).get("p95", 0)
        if mpc_p95 > tg.get("mpc_p95_max_us", 1e9):
            ok = False
        if cyc_p95 > tg.get("cycle_p95_max_us", 1e9):
            ok = False
        if t.get("deadline_miss", 0) > tg.get("deadline_miss_max", 0):
            ok = False

    return ok


def _print_summary(m: dict, th: dict) -> None:
    vb = th.get("vs_baseline", {})
    tg = th.get("timing", {})

    def _chk(name, val, limit, fmt=".3f"):
        if val is None:
            return
        if limit is None:
            print(f"      {name:22s} = {val:{fmt}}")
            return
        ok = val <= limit
        mark = "✓" if ok else "✗"
        print(f"    {mark} {name:22s} = {val:{fmt}}   (limit {limit})")

    print()
    print("  [baseline ↔ PIL]")
    _chk("velocity_mae (m/s)",    m.get("velocity_mae"),    vb.get("velocity_mae_max"))
    _chk("attitude_max (deg)",    m.get("attitude_max_deg"), vb.get("attitude_max_deg"))
    _chk("altitude_mae (m)",      m.get("altitude_mae"),    vb.get("altitude_mae_max"))
    _chk("altitude_max_err (m)",  m.get("altitude_max_err"), None)
    _chk("range_mae (m)",         m.get("range_mae"),       None)
    _chk("CEP (m)",               m.get("cep_m"),           vb.get("cep_max_m"))

    t = m.get("timing", {})
    if t and t.get("samples", 0) > 0:
        print(f"\n  [timing] samples={t['samples']}  "
              f"deadline_miss={t['deadline_miss']}  "
              f"(limit {tg.get('deadline_miss_max', 0)})")
        for stage in ("mhe_us", "mpc_us", "cycle_us"):
            s = t.get(stage, {})
            if s:
                print(f"    {stage:10s} p50={s['p50']:7.1f}  "
                      f"p95={s['p95']:7.1f}  p99={s['p99']:7.1f}  "
                      f"max={s['max']:7.1f}")
    elif t:
        print("\n  [timing] no samples — verify target publishes timing messages")

    print()
    print(f"  RESULT: {'PASS ✓' if m.get('pass') else 'FAIL ✗'}")
    print("=" * 70)


# ============================================================================
# CLI
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="M130 PIL Test Runner (standalone)")
    ap.add_argument("--config", default=str(_SCRIPT_DIR / "pil_config.yaml"))
    ap.add_argument("--baseline-only", action="store_true")
    ap.add_argument("--pil-only", action="store_true")
    ap.add_argument("--compare-only", action="store_true")
    ap.add_argument("--baseline-csv", default=None)
    ap.add_argument("--pil-csv", default=None)
    ap.add_argument("--timing-csv", default=None)
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    out = cfg.get("output", {})
    baseline_csv = args.baseline_csv or str(_RESULTS / "baseline_flight.csv")
    pil_csv = args.pil_csv or str(_RESULTS / out.get("csv_name", "pil_flight.csv"))
    timing_csv = args.timing_csv or str(_RESULTS / out.get("timing_csv", "pil_timing.csv"))
    os.makedirs(_RESULTS, exist_ok=True)

    if args.baseline_only:
        run_baseline(output_csv=baseline_csv)
        return

    if args.pil_only:
        run_pil(args.config, pil_csv, timing_csv)
        return

    if not args.compare_only:
        if not Path(baseline_csv).exists():
            run_baseline(output_csv=baseline_csv)
        else:
            print(f"[runner] reusing baseline: {baseline_csv}")
        run_pil(args.config, pil_csv, timing_csv)

    report = compare(baseline_csv, pil_csv, timing_csv, cfg["thresholds"])
    sys.exit(0 if report.get("pass") else 1)


if __name__ == "__main__":
    main()
