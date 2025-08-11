#!/usr/bin/env python3
"""Run the TRIAD initialisation on a single fixed IMU/GNSS dataset.

This script mirrors :mod:`run_all_methods.py` but processes only
``IMU_X002.dat`` together with ``GNSS_X002.csv``.  The full GNSS/IMU fusion
pipeline (Tasks 1--7) is executed and all outputs are collected under
``results/IMU_X002_GNSS_X002_TRIAD``.  Use this helper to compare the Python and
MATLAB pipelines on a single dataset before running the full batch.

Usage
-----
    python src/run_triad_only.py [options]
"""

from __future__ import annotations

import os
import sys

sys.path.append(os.path.join(os.getcwd(), "src", "utils"))
from trace_utils import (
    set_debug,
    log_msg,
    trace_task,
    dump_structure,
    save_plot_interactive,
)

set_debug(os.getenv("DEBUG", "").lower() in ("1", "true"))
log_msg("run_triad_only.py loaded")

import argparse
import json
import logging
import math
import pathlib
import re
import subprocess
import time
import io
from contextlib import redirect_stdout
from pathlib import Path
from typing import Iterable, List, Dict

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import scipy.io as sio
from tabulate import tabulate

from evaluate_filter_results import run_evaluation_npz
from run_all_methods import run_case, compute_C_NED_to_ECEF
from utils import save_mat
from timeline import print_timeline
from resolve_truth_path import resolve_truth_path
from run_id import run_id as build_run_id

sys.path.append(str(Path(__file__).resolve().parents[1] / "tools"))
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def check_files(
    imu_file: str, gnss_file: str
) -> tuple[pathlib.Path, pathlib.Path]:
    """Return validated paths for the IMU and GNSS files."""

    imu_path = pathlib.Path(imu_file)
    gnss_path = pathlib.Path(gnss_file)
    return imu_path, gnss_path


def _unwrap_clock_1s(t_raw, wrap=1.0, tol=0.25):
    """Unwrap a clock that resets every ``wrap`` seconds (default 1s)."""
    import numpy as np
    t = np.asarray(t_raw, dtype=float).ravel()
    if t.size == 0:
        return t, 0
    out = t.copy()
    wraps = 0
    offset = 0.0
    for i in range(1, t.size):
        if t[i] + 1e-12 < t[i - 1] - tol:
            wraps += 1
            offset += wrap
        out[i] = t[i] + offset
    return out, wraps


def _make_monotonic_time(t_like, fallback_len=None, dt=None, imu_rate_hint=None):
    """Build a strictly increasing IMU timebase."""
    import numpy as np
    if t_like is None or (hasattr(t_like, "__len__") and len(t_like) == 0):
        if dt is None:
            if imu_rate_hint and imu_rate_hint > 0:
                dt = 1.0 / float(imu_rate_hint)
            else:
                raise ValueError("No IMU timestamps and no dt/imu_rate_hint provided.")
        n = int(fallback_len) if fallback_len is not None else 0
        t = np.arange(n, dtype=float) * float(dt)
        return t, {"source": "synth", "wraps": 0, "dt": dt}

    t = np.asarray(t_like, dtype=float).ravel()
    meta = {"source": "file", "wraps": 0}
    span = float(t.max() - t.min()) if t.size else 0.0
    if span < 2.0 and t.size > 2000:
        t, wraps = _unwrap_clock_1s(t, wrap=1.0, tol=0.25)
        meta["wraps"] = int(wraps)
        meta["unwrapped_span"] = float(t[-1] - t[0]) if t.size else 0.0
        print(
            f"[Clock] Detected 1s-resetting IMU clock; unwrapped with {wraps} wraps -> {meta['unwrapped_span']:.2f}s total."
        )
    d = np.diff(t)
    if (d <= 0).any():
        eps = np.finfo(float).eps
        t = t + np.arange(t.size) * eps
        meta["jitter_eps_applied"] = True
    return t, meta


def _write_run_meta(outdir, run_id, **kv):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    meta_path = outdir / f"{run_id}_runmeta.json"
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(kv, f, indent=2, sort_keys=True)
    print(f"Saved run meta -> {meta_path}")


def main(argv: Iterable[str] | None = None) -> None:
    results_dir = pathlib.Path("results")
    results_dir.mkdir(parents=True, exist_ok=True)
    logger.info("Ensured '%s' directory exists.", results_dir)
    print("Note: Python saves to results/ ; MATLAB saves to MATLAB/results/ (independent).")

    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with the TRIAD method on the X002 dataset",
    )
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    parser.add_argument(
        "--show-measurements",
        action="store_true",
        help="Include IMU and GNSS measurements in Task 6 overlay plots",
    )
    parser.add_argument(
        "--task",
        type=int,
        help="Run a single helper task and exit",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    parser.add_argument("--imu-rate", type=float, default=None, help="Hint IMU sample rate [Hz]")
    parser.add_argument("--gnss-rate", type=float, default=None, help="Hint GNSS sample rate [Hz]")
    parser.add_argument("--truth-rate", type=float, default=None, help="Hint truth sample rate [Hz]")

    args = parser.parse_args(argv)

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    if args.task == 7:
        from evaluate_filter_results import run_evaluation

        # Save Task 7 output directly under ``results/``
        run_evaluation(
            prediction_file="outputs/predicted_states.csv",
            gnss_file="outputs/gnss_measurements.csv",
            attitude_file="outputs/estimated_attitude.csv",
            save_path="results",
        )
        return

    method = "TRIAD"

    results: List[Dict[str, float | str]] = []

    imu_file = "IMU_X002.dat"
    gnss_file = "GNSS_X002.csv"

    imu_path, gnss_path = check_files(imu_file, gnss_file)

    run_id = build_run_id(str(imu_path), str(gnss_path), method)
    log_path = results_dir / f"{run_id}.log"
    print(f"\u25b6 {run_id}")

    truth_path = resolve_truth_path()

    print("Note: Python saves to results/ ; MATLAB saves to MATLAB/results/ (independent).")
    print_timeline(run_id, str(imu_path), str(gnss_path), truth_path, out_dir=str(results_dir))

    if logger.isEnabledFor(logging.DEBUG):
        try:
            gnss_preview = np.loadtxt(gnss_path, delimiter=",", skiprows=1, max_rows=1)
            imu_preview = np.loadtxt(imu_path, max_rows=1)
            logger.debug(
                "GNSS preview: shape %s, first row: %s",
                gnss_preview.shape,
                gnss_preview,
            )
            logger.debug(
                "IMU preview: shape %s, first row: %s",
                imu_preview.shape,
                imu_preview,
            )
        except Exception as e:  # pragma: no cover - best effort
            logger.warning(
                "Failed data preview for %s or %s: %s", imu_file, gnss_file, e
            )

    cmd = [
        sys.executable,
        str(HERE / "GNSS_IMU_Fusion.py"),
        "--imu-file",
        str(imu_path),
        "--gnss-file",
        str(gnss_path),
        "--method",
        method,
    ]
    if args.no_plots:
        cmd.append("--no-plots")

    start_t = time.time()
    ret, summaries = trace_task("Pipeline")(run_case)(cmd, log_path)
    runtime = time.time() - start_t
    if ret != 0:
        raise subprocess.CalledProcessError(ret, cmd)

    # Move generated files when a separate results directory was used in older
    # versions. With the flat layout ``results_dir`` equals ``base_results`` so
    # the loop becomes a no-op.
    base_results = pathlib.Path("results")
    if results_dir != base_results:
        for file in base_results.glob(f"{run_id}*"):
            dest = results_dir / file.name
            try:
                file.replace(dest)
            except Exception:
                pass

    for summary in summaries:
        kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
        results.append(
            {
                "dataset": pathlib.Path(imu_file).stem,
                "method": kv.get("method", method),
                "rmse_pos": float(kv.get("rmse_pos", "nan").replace("m", "")),
                "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                "rms_resid_pos": float(kv.get("rms_resid_pos", "nan").replace("m", "")),
                "max_resid_pos": float(kv.get("max_resid_pos", "nan").replace("m", "")),
                "rms_resid_vel": float(kv.get("rms_resid_vel", "nan").replace("m", "")),
                "max_resid_vel": float(kv.get("max_resid_vel", "nan").replace("m", "")),
                "runtime": runtime,
            }
        )

    # ------------------------------------------------------------------
    # Convert NPZ output to a MATLAB file with explicit frame variables
    # ------------------------------------------------------------------
    npz_path = results_dir / f"{run_id}_kf_output.npz"
    t_imu = None
    tmeta: Dict[str, float | int | str] = {}
    if npz_path.exists():
        data = np.load(npz_path, allow_pickle=True)
        dump_structure("Task5.output", data)
        logger.debug("Loaded output %s with keys: %s", npz_path, list(data.keys()))

        t3_path = results_dir / f"{run_id}_task3_results.mat"
        if t3_path.exists():
            t3 = sio.loadmat(str(t3_path), simplify_cells=True)
            if "R" in t3:
                R = t3["R"]
            elif "Rbn" in t3:
                R = t3["Rbn"]
            elif "Task3" in t3 and isinstance(t3["Task3"], dict) and "R" in t3["Task3"]:
                R = t3["Task3"]["R"]
            else:
                logger.warning(
                    "[Task_5] No rotation field found (R/Rbn). Available: %s",
                    list(t3.keys()),
                )
                dump_structure("Task5.Task3_loaded", t3, max_depth=3)
                R = None
            if R is not None:
                t3["R"] = R
            dump_structure("Task5.Task3_loaded", t3, max_depth=3)
        time_s_raw = data.get("time_s")
        if time_s_raw is None:
            time_s_raw = data.get("time")

        dt_hint = None
        for name in ("imu_dt", "dt_imu", "dt"):
            if name in data and np.isscalar(data[name]):
                dt_hint = float(data[name])
                break

        imu_rate_hint = args.imu_rate
        if imu_rate_hint is None:
            for name in ("IMU_RATE_HZ", "imu_rate", "rate_imu_hz", "rate_imu", "imu_rate_hz"):
                if name in data and np.isscalar(data[name]):
                    imu_rate_hint = float(data[name])
                    break

        t_imu, tmeta = _make_monotonic_time(
            t_like=time_s_raw,
            fallback_len=len(time_s_raw) if time_s_raw is not None else None,
            dt=dt_hint,
            imu_rate_hint=imu_rate_hint,
        )
        time_s = t_imu

        if time_s is not None and len(time_s) > 1:
            imu_dt = float(np.median(np.diff(time_s)))
            imu_rate_hz = 1.0 / imu_dt
        else:
            imu_dt = None
            imu_rate_hz = imu_rate_hint

        pos_ned = data.get("pos_ned_m")
        if pos_ned is None:
            pos_ned = data.get("pos_ned")
        if pos_ned is None:
            pos_ned = data.get("fused_pos")

        vel_ned = data.get("vel_ned_ms")
        if vel_ned is None:
            vel_ned = data.get("vel_ned")
        if vel_ned is None:
            vel_ned = data.get("fused_vel")

        if pos_ned is not None:
            dump_structure("Task5.pos_ned", pos_ned)
        if vel_ned is not None:
            dump_structure("Task5.vel_ned", vel_ned)

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "Output time range: %s to %s s",
                time_s[0] if time_s is not None else "N/A",
                time_s[-1] if time_s is not None else "N/A",
            )
            logger.debug(
                "Position shape: %s",
                pos_ned.shape if pos_ned is not None else "None",
            )

        ref_lat = data.get("ref_lat_rad")
        ref_lon = data.get("ref_lon_rad")
        ref_r0 = data.get("ref_r0_m")
        if ref_lat is None:
            ref_lat = data.get("ref_lat")
        if ref_lon is None:
            ref_lon = data.get("ref_lon")
        if ref_r0 is None:
            ref_r0 = data.get("ref_r0")
        ref_lat = float(np.squeeze(ref_lat))
        ref_lon = float(np.squeeze(ref_lon))
        ref_r0 = np.asarray(ref_r0)

        C_NED_ECEF = compute_C_NED_to_ECEF(ref_lat, ref_lon)
        pos_ecef = (C_NED_ECEF @ pos_ned.T).T + ref_r0
        vel_ecef = (C_NED_ECEF @ vel_ned.T).T

        quat = data.get("att_quat")
        if quat is None:
            quat = data.get("attitude_q")
        if quat is None:
            quat = data.get("quat_log")
        if quat is not None:
            rot = R.from_quat(quat[:, [1, 2, 3, 0]])
            C_B_N = rot.as_matrix()
            pos_body = np.einsum("nij,nj->ni", C_B_N.transpose(0, 2, 1), pos_ned)
            vel_body = np.einsum("nij,nj->ni", C_B_N.transpose(0, 2, 1), vel_ned)
        else:
            pos_body = np.zeros_like(pos_ned)
            vel_body = np.zeros_like(vel_ned)

        mat_out = {
            "time_s": time_s,
            "t_est": time_s,
            "pos_ned_m": pos_ned,
            "vel_ned_ms": vel_ned,
            "pos_ecef_m": pos_ecef,
            "vel_ecef_ms": vel_ecef,
            "pos_body_m": pos_body,
            "vel_body_ms": vel_body,
            "ref_lat_rad": ref_lat,
            "ref_lon_rad": ref_lon,
            "ref_r0_m": ref_r0,
            "att_quat": quat,
            "method_name": method,
            "dt": imu_dt,
            "imu_rate_hz": imu_rate_hz,
            # Consistent fused data for Tasks 5 and 6
            "fused_pos": pos_ned,
            "fused_vel": vel_ned,
            "quat_log": quat,
        }
        save_mat(npz_path.with_suffix(".mat"), mat_out)

        # ------------------------------------------------------------------
        # Export estimator time vector and sampling interval for MATLAB Task 7
        # ------------------------------------------------------------------
        x_log = data.get("x_log")
        if x_log is not None and imu_dt is not None:
            t_est = np.arange(x_log.shape[1]) * imu_dt
            mat_time = {"t_est": t_est, "dt": imu_dt, "x_log": x_log}
            time_path = results_dir / f"{run_id}_task5_time.mat"
            sio.savemat(str(time_path), mat_time)
            logger.info("Saved Task 5 time data to %s", time_path)

        logger.info(
            "Subtask 6.8.2: Plotted %s position North: First = %.4f, Last = %.4f",
            method,
            pos_ned[0, 0],
            pos_ned[-1, 0],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s position East: First = %.4f, Last = %.4f",
            method,
            pos_ned[0, 1],
            pos_ned[-1, 1],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s position Down: First = %.4f, Last = %.4f",
            method,
            pos_ned[0, 2],
            pos_ned[-1, 2],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s velocity North: First = %.4f, Last = %.4f",
            method,
            vel_ned[0, 0],
            vel_ned[-1, 0],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s velocity East: First = %.4f, Last = %.4f",
            method,
            vel_ned[0, 1],
            vel_ned[-1, 1],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s velocity Down: First = %.4f, Last = %.4f",
            method,
            vel_ned[0, 2],
            vel_ned[-1, 2],
        )
        def _infer_rate(t):
            import numpy as np
            if t is None or len(t) < 2:
                return None
            dt = np.median(np.diff(t))
            return None if dt <= 0 else 1.0 / float(dt)

        meta = {
            "imu_file": str(imu_path),
            "gnss_file": str(gnss_path),
            "truth_file": str(ROOT / "STATE_X001.txt") if (ROOT / "STATE_X001.txt").exists() else None,
            "imu_rate_hz": _infer_rate(t_imu) or args.imu_rate,
            "gnss_rate_hz": args.gnss_rate,
            "truth_rate_hz": args.truth_rate,
            "imu_time_meta": tmeta,
        }
        _write_run_meta("results", run_id, **meta)

    # ----------------------------
    # Task 6: Truth overlay plots
    # ----------------------------
    truth_file = ROOT / "STATE_X001.txt"
    if truth_file.exists():
        overlay_cmd = [
            sys.executable,
            str(HERE / "task6_plot_truth.py"),
            "--est-file",
            str(npz_path.with_suffix(".mat")),
            "--imu-file",
            str(imu_path),
            "--gnss-file",
            str(gnss_path),
            "--truth-file",
            str(truth_file.resolve()),
            "--output",
            str(results_dir),
        ]
        if args.show_measurements:
            overlay_cmd.append("--show-measurements")
        with open(log_path, "a") as log:
            log.write("\nTASK 6: Overlay fused output with truth\n")
            msg = "Starting Task 6 overlay ..."
            logger.info(msg)
            log.write(msg + "\n")
            proc = subprocess.Popen(
                overlay_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            for line in proc.stdout:
                print(line, end="")
                log.write(line)
            proc.wait()

    # ----------------------------
    # Task 7: Evaluation
    # ----------------------------
    # Task 7 plots are now saved directly under the ``results``
    # directory without subfolders for easier navigation.
    task7_dir = results_dir
    with open(log_path, "a") as log:
        log.write("\nTASK 7: Evaluate residuals\n")
        msg = "Running Task 7 evaluation ..."
        logger.info(msg)
        log.write(msg + "\n")
        buf = io.StringIO()
        with redirect_stdout(buf):
            try:
                run_evaluation_npz(str(npz_path), str(task7_dir), run_id)
            except Exception as e:  # pragma: no cover - graceful failure
                print(f"Task 7 failed: {e}")
        output = buf.getvalue()
        print(output, end="")
        log.write(output)
        print(
            f"Saved Task 7.5 diff-truth plots (NED/ECEF/Body) under: results/{run_id}/"
        )

    # --- nicely formatted summary table --------------------------------------
    if results:
        rows = [
            [
                e["dataset"],
                e["method"],
                e["rmse_pos"],
                e["final_pos"],
                e["rms_resid_pos"],
                e["max_resid_pos"],
                e["rms_resid_vel"],
                e["max_resid_vel"],
                e["runtime"],
            ]
            for e in results
        ]
        print(
            tabulate(
                rows,
                headers=[
                    "Dataset",
                    "Method",
                    "RMSEpos [m]",
                    "End-Error [m]",
                    "RMSresidPos [m]",
                    "MaxresidPos [m]",
                    "RMSresidVel [m/s]",
                    "MaxresidVel [m/s]",
                    "Runtime [s]",
                ],
                floatfmt=".2f",
            )
        )
        df = pd.DataFrame(
            rows,
            columns=[
                "Dataset",
                "Method",
                "RMSEpos_m",
                "EndErr_m",
                "RMSresidPos_m",
                "MaxresidPos_m",
                "RMSresidVel_mps",
                "MaxresidVel_mps",
                "Runtime_s",
            ],
        )
        df.to_csv(results_dir / "summary.csv", index=False)

    print("TRIAD processing complete for X002")


if __name__ == "__main__":
    main()

