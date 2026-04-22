#!/usr/bin/env python3
"""
HIL Test Runner — مستقل تماماً
==============================

يُشغّل جسر MAVLink HIL (mavlink_bridge_hil) الذي يتصل بجهاز ARM64
خارجي (Android + PX4)، ثم يُحلّل السيرفو والتوقيت ضد عتبات hil_config.

المقارنة ضد baseline Python أُزيلت كلياً من هذا العدّاء؛ البوابات
الوحيدة الآن: servo_tracking + timing على مخرجات HIL نفسها.

Usage:
    python hil_runner.py                        # HIL + تحليل
    python hil_runner.py --hil-only             # HIL فقط (بدون تحليل)
    python hil_runner.py --compare-only         # تحليل من CSV موجودة
"""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
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
# HIL
# ============================================================================

_APP_PACKAGE = "com.ardophone.px4v17"
_APP_MAIN_ACTIVITY = f"{_APP_PACKAGE}/.MainActivity"


def _adb() -> str | None:
    """يحاول تحديد مسار adb. يُرجع None إن لم يُعثر عليه."""
    for cand in (
        shutil.which("adb"),
        "/home/yoga/Android/Sdk/platform-tools/adb",
        os.path.expanduser("~/Android/Sdk/platform-tools/adb"),
    ):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


def _run_adb(adb: str, args: list[str], check: bool = False,
             timeout: float = 10.0) -> subprocess.CompletedProcess:
    """نفّذ أمر adb ويُرجع النتيجة مع stdout/stderr."""
    return subprocess.run(
        [adb] + args,
        check=check, timeout=timeout,
        capture_output=True, text=True,
    )


def preflight_reset(cfg_path: str) -> None:
    """تهيئة بيئة نظيفة قبل تشغيل HIL.

    الخطوات:
      1) force-stop تطبيق PX4 على الهاتف (يُزيل أي حالة armed/termination عالقة
         وأي sockets مسرّبة على port 5760)
      2) إزالة جميع adb reverse/forward القديمة
      3) إنشاء reverse 4560 و forward 5760 من جديد
      4) إعادة تشغيل التطبيق وانتظار 5s لتهيئة PX4 modules

    يُتخطى بالكامل إذا warmup.preflight_reset=false أو adb غير متاح.
    """
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        return
    if not bool(cfg.get("warmup", {}).get("preflight_reset", True)):
        print("[preflight] skipped (warmup.preflight_reset=false)")
        return

    adb = _adb()
    if adb is None:
        print("[preflight] WARNING: adb not found in PATH — skipping reset. "
              "Ensure ports 4560/5760 are reachable manually.")
        return

    print("[preflight] Resetting phone + adb ports...")
    devs = _run_adb(adb, ["devices"]).stdout
    if "\tdevice" not in devs:
        print("[preflight] WARNING: no adb device connected — skipping reset.")
        return

    _run_adb(adb, ["shell", "am", "force-stop", _APP_PACKAGE])
    time.sleep(1.5)
    _run_adb(adb, ["reverse", "--remove-all"])
    _run_adb(adb, ["forward", "--remove-all"])
    r1 = _run_adb(adb, ["reverse", "tcp:4560", "tcp:4560"])
    r2 = _run_adb(adb, ["forward", "tcp:5760", "tcp:5760"])
    if r1.returncode != 0 or r2.returncode != 0:
        print(f"[preflight] WARNING: adb reverse/forward failed: "
              f"reverse={r1.stderr.strip()} forward={r2.stderr.strip()}")
    _run_adb(adb, ["shell", "am", "start", "-n", _APP_MAIN_ACTIVITY])
    print("[preflight] App launched — waiting 5s for PX4 modules to initialize...")
    time.sleep(5.0)
    print("[preflight] Ready.")


def run_hil(cfg_path: str, flight_csv: str, timing_csv: str) -> tuple[str, str]:
    """يُطلق جسر HIL المستقل وينتظر اتصال الهدف."""
    from mavlink_bridge_hil import HILBridge

    preflight_reset(cfg_path)

    print("=" * 70)
    print("  HIL: connecting to remote target")
    print("=" * 70)

    bridge = HILBridge(cfg_path)
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
    "fin_act_1": ["fin_act_1"],
    "fin_act_2": ["fin_act_2"],
    "fin_act_3": ["fin_act_3"],
    "fin_act_4": ["fin_act_4"],
    "fin_can_1": ["fin_can_1"],
    "fin_can_2": ["fin_can_2"],
    "fin_can_3": ["fin_can_3"],
    "fin_can_4": ["fin_can_4"],
    "fin_source": ["fin_source"],
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
    str_cols: dict[str, list[str]] = {}
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for k in r.fieldnames or []:
            raw[k] = []
            str_cols[k] = []
        for row in r:
            for k in r.fieldnames or []:
                try:
                    raw[k].append(float(row[k]))
                except (ValueError, TypeError):
                    raw[k].append(0.0)
                    str_cols[k].append(row[k] if row[k] else "")
    data = {k: np.array(v) for k, v in raw.items()}
    for k, vals in str_cols.items():
        if any(v != "" for v in vals):
            data[f"_str_{k}"] = vals
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
# التحليل: HIL وحدها (servo + timing)
# ============================================================================

def compare(hil_csv: str, timing_csv: str | None,
            thresholds: dict, deadline_us: int = 20000) -> dict:
    print()
    print("=" * 70)
    print("  HIL analysis — servo + timing")
    print("=" * 70)

    hil = load_csv(hil_csv)
    t_p = hil.get("time", np.array([]))
    if len(t_p) == 0:
        print("  ERROR: empty HIL data")
        return {"pass": False, "error": "empty"}

    m: dict = {"t_final_hil": float(t_p[-1])}

    # تتبع السيرفو (CAN feedback vs أمر MPC)
    servo = _analyze_servo_tracking(hil)
    m["servo_tracking"] = servo

    # التوقيت
    timing = _analyze_timing(timing_csv, deadline_us) if timing_csv and Path(timing_csv).exists() else {}
    m["timing"] = timing

    # بوابة
    m["pass"] = _gate(m, thresholds)

    _print_summary(m, thresholds)
    return m


def _analyze_servo_tracking(hil_data: dict) -> dict:
    """يحلل تتبع السيرفو: fin_act (زاوية العتاد المقاسة) ↔ fin_cmd (أمر MPC).

    في closed_loop HIL (الوضع الوحيد)، fin_act = الزاوية الحقيقية
    المقاسة من السيرفوهات عبر SRV_FB (مطابق لـ fin_can). العتبة تقيس
    أداء السيرفو الحقيقي (backlash/slew/CAN-latency/jitter).
    """
    has_act = all(k in hil_data for k in ("fin_act_1", "fin_act_2", "fin_act_3", "fin_act_4"))
    has_cmd = all(k in hil_data for k in ("fin_cmd_1", "fin_cmd_2", "fin_cmd_3", "fin_cmd_4"))

    if not has_act or not has_cmd:
        return {"available": False}

    n = len(hil_data["fin_cmd_1"])
    if n == 0:
        return {"available": False}

    act_rad = np.column_stack([hil_data["fin_act_1"], hil_data["fin_act_2"],
                               hil_data["fin_act_3"], hil_data["fin_act_4"]])
    cmd_rad = np.column_stack([hil_data["fin_cmd_1"], hil_data["fin_cmd_2"],
                               hil_data["fin_cmd_3"], hil_data["fin_cmd_4"]])

    # نحسب MAE/P95 فقط على الخطوات التي fin_source == "can" (فيدباك حيّ)
    # — خطوات hold/cmd/abort تحتوي على fin_act مُطابق للأمر (err=0) أو
    # قديم، ما يُلوّث إحصاء أداء العتاد الحقيقي.
    # load_csv يضع القيم النصية في "_str_fin_source" وأصفاراً في "fin_source"
    # (لأن float("can") يرفع) — نُفضّل النسخة النصية.
    fin_source = hil_data.get("_str_fin_source") or hil_data.get("fin_source")
    if fin_source is not None and len(fin_source) == n:
        can_mask = np.asarray(
            [str(s).strip() == "can" for s in fin_source], dtype=bool
        )
    else:
        can_mask = np.ones(n, dtype=bool)

    # خطأ التتبع: fin_act يجب أن يتبع fin_cmd مع تأخير العتاد الفيزيائي
    err_deg = np.abs(np.degrees(act_rad - cmd_rad))
    max_err_per_step = np.max(err_deg, axis=1)
    can_steps = int(can_mask.sum())
    if can_steps > 0:
        tracked_err = max_err_per_step[can_mask]
    else:
        # لا توجد خطوات can (جلسة انتهت في grace أو abort) — نُعيد تقريراً
        # شفافاً يوضح السبب بدل إحصاءات مضلِّلة.
        tracked_err = max_err_per_step

    # معلومات CAN (للعرض فقط — لا يُستخدم في البوابة)
    can_info = {}
    has_can = all(k in hil_data for k in ("fin_can_1", "fin_can_2", "fin_can_3", "fin_can_4"))
    if has_can:
        can_rad = np.column_stack([hil_data["fin_can_1"], hil_data["fin_can_2"],
                                   hil_data["fin_can_3"], hil_data["fin_can_4"]])
        can_diff = np.diff(can_rad, axis=0)
        can_info["can_frames"] = int(np.sum(np.any(np.abs(can_diff) > 1e-6, axis=1))) + 1
        can_cmd_err = np.abs(np.degrees(can_rad - cmd_rad))
        can_info["can_cmd_mae_deg"] = float(np.mean(np.max(can_cmd_err, axis=1)))

    result = {
        "available": True,
        "total_steps": n,
        "can_steps": can_steps,
        "tracking_mae_deg": float(np.mean(tracked_err)),
        "tracking_p95_deg": float(np.percentile(tracked_err, 95)),
        "tracking_p99_deg": float(np.percentile(tracked_err, 99)),
        "tracking_max_deg": float(np.max(tracked_err)),
    }
    result.update(can_info)
    return result


def _analyze_timing(path: str, deadline_us: int = 20000) -> dict:
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
        "deadline_miss": int(sum(1 for c in cyc if c > deadline_us)),
        "jitter_std_us": float(np.std(cyc)) if len(cyc) > 1 else 0.0,
    }


def _gate(m: dict, th: dict) -> bool:
    ok = True
    tg = th.get("timing", {})
    st = th.get("servo_tracking", {})

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
        mpc_p99 = t.get("mpc_us", {}).get("p99", 0)
        if mpc_p99 > tg.get("mpc_p99_max_us", 1e9):
            ok = False
        jitter = t.get("jitter_std_us", 0)
        if jitter > tg.get("jitter_std_max_us", 1e9):
            ok = False

    # بوابة تتبع السيرفو (fin_act ↔ fin_cmd — العتاد الحقيقي)
    srv = m.get("servo_tracking", {})
    if srv.get("available", False):
        if srv.get("tracking_mae_deg", 0) > st.get("tracking_mae_max_deg", 1e9):
            ok = False
        if srv.get("tracking_p95_deg", 0) > st.get("tracking_p95_max_deg", 1e9):
            ok = False

    return ok


def _print_summary(m: dict, th: dict) -> None:
    tg = th.get("timing", {})
    st = th.get("servo_tracking", {})

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

    # تتبع السيرفو (fin_act ↔ fin_cmd — العتاد الحقيقي)
    srv = m.get("servo_tracking", {})
    if srv.get("available", False):
        print(f"\n  [servo tracking] fin_act vs fin_cmd   steps={srv.get('total_steps',0)}")
        if srv.get("can_frames") is not None:
            print(f"    CAN frames={srv['can_frames']}  "
                  f"CAN-cmd MAE={srv.get('can_cmd_mae_deg',0):.2f}°  "
                  f"(display only — PX4 HIL does not drive CAN)")
        _chk("tracking_mae (deg)",    srv.get("tracking_mae_deg"),    st.get("tracking_mae_max_deg"))
        _chk("tracking_p95 (deg)",     srv.get("tracking_p95_deg"),    st.get("tracking_p95_max_deg"))
        _chk("tracking_p99 (deg)",     srv.get("tracking_p99_deg"),    None)
        _chk("tracking_max (deg)",     srv.get("tracking_max_deg"),    None)
    else:
        print("\n  [servo tracking] no fin_act/fin_cmd data in HIL CSV")

    t = m.get("timing", {})
    if t and t.get("samples", 0) > 0:
        print(f"\n  [timing] samples={t['samples']}  "
              f"deadline_miss={t['deadline_miss']}  "
              f"(limit {tg.get('deadline_miss_max', 0)})  "
              f"jitter_std={t.get('jitter_std_us', 0):.1f}µs")
        for stage in ("mhe_us", "mpc_us", "cycle_us"):
            s = t.get(stage, {})
            if s:
                print(f"    {stage:10s} p50={s['p50']:7.1f}  "
                      f"p95={s['p95']:7.1f}  p99={s['p99']:7.1f}  "
                      f"max={s['max']:7.1f}")
        _chk("mpc_p99 (µs)", t.get("mpc_us", {}).get("p99"), tg.get("mpc_p99_max_us"), fmt=".1f")
        _chk("jitter_std (µs)", t.get("jitter_std_us"), tg.get("jitter_std_max_us"), fmt=".1f")
    elif t:
        print("\n  [timing] no samples — verify target publishes timing messages")

    print()
    print(f"  RESULT: {'PASS ✓' if m.get('pass') else 'FAIL ✗'}")
    print("=" * 70)


# ============================================================================
# CLI
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="M130 HIL Test Runner (standalone)")
    ap.add_argument("--config", default=str(_SCRIPT_DIR / "hil_config.yaml"))
    ap.add_argument("--hil-only", action="store_true",
                    help="شغّل جسر HIL فقط بدون تحليل لاحق")
    ap.add_argument("--compare-only", action="store_true",
                    help="حلّل CSV موجودة بدون تشغيل HIL")
    ap.add_argument("--hil-csv", default=None)
    ap.add_argument("--timing-csv", default=None)
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    out = cfg.get("output", {})
    hil_csv = args.hil_csv or str(_RESULTS / out.get("csv_name", "hil_flight.csv"))
    timing_csv = args.timing_csv or str(_RESULTS / out.get("timing_csv", "hil_timing.csv"))
    os.makedirs(_RESULTS, exist_ok=True)

    if args.hil_only:
        run_hil(args.config, hil_csv, timing_csv)
        return

    if not args.compare_only:
        run_hil(args.config, hil_csv, timing_csv)

    deadline_us = int(cfg.get("timing", {}).get("deadline_us", 20000))
    report = compare(hil_csv, timing_csv, cfg["thresholds"],
                     deadline_us=deadline_us)
    sys.exit(0 if report.get("pass") else 1)


if __name__ == "__main__":
    main()
