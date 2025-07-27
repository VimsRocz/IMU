#!/usr/bin/env python3
"""Run the TRIAD initialisation on a single fixed IMU/GNSS dataset.

This script mirrors :mod:`run_all_methods.py` but is simplified to process only
``IMU_X001.dat`` together with ``GNSS_X001.csv``.  The full GNSS/IMU fusion
pipeline (Tasks 1--7) is executed and all outputs are collected under
``results/IMU_X001_GNSS_X001_TRIAD``.  Use this helper to compare the Python and
MATLAB pipelines on a single dataset before running the full batch.

Usage
-----
    python src/run_triad_only.py [options]
"""

from __future__ import annotations

import argparse
import logging
import os
import pathlib
import re
import subprocess
import sys
import time
import io
from contextlib import redirect_stdout
from typing import Iterable, List, Dict

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from tabulate import tabulate

from evaluate_filter_results import run_evaluation_npz
from run_all_methods import run_case, compute_C_NED_to_ECEF
from utils import save_mat

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def main(argv: Iterable[str] | None = None) -> None:
    results_dir = pathlib.Path("results/IMU_X001_GNSS_X001_TRIAD")
    results_dir.mkdir(parents=True, exist_ok=True)
    logger.info("Ensured '%s' directory exists.", results_dir)

    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with the TRIAD method on the X001 dataset",
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

    args = parser.parse_args(argv)

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    if args.task == 7:
        from evaluate_filter_results import run_evaluation

        run_evaluation(
            prediction_file="outputs/predicted_states.csv",
            gnss_file="outputs/gnss_measurements.csv",
            attitude_file="outputs/estimated_attitude.csv",
            save_path="plots/task7/",
        )
        return

    method = "TRIAD"
    data_dir = ROOT / "Data"
    if not data_dir.is_dir():
        data_dir = ROOT

    results: List[Dict[str, float | str]] = []

    imu = "data/IMU_X001.dat"
    gnss = "data/GNSS_X001.csv"

    tag = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{method}"
    log_path = results_dir / f"{tag}.log"
    print(f"\u25b6 {tag}")

    imu_path = data_dir / imu
    gnss_path = data_dir / gnss
    if not imu_path.is_file():
        imu_path = ROOT / imu
    if not gnss_path.is_file():
        gnss_path = ROOT / gnss

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
            logger.warning("Failed data preview for %s or %s: %s", imu, gnss, e)

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
    ret, summaries = run_case(cmd, log_path)
    runtime = time.time() - start_t
    if ret != 0:
        raise subprocess.CalledProcessError(ret, cmd)

    # Move generated result files into the dedicated directory
    base_results = pathlib.Path("results")
    for file in base_results.glob(f"{tag}*"):
        dest = results_dir / file.name
        try:
            file.replace(dest)
        except Exception:
            pass

    for summary in summaries:
        kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
        results.append(
            {
                "dataset": pathlib.Path(imu).stem,
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
    npz_path = results_dir / f"{tag}_kf_output.npz"
    if npz_path.exists():
        data = np.load(npz_path, allow_pickle=True)
        logger.debug("Loaded output %s with keys: %s", npz_path, list(data.keys()))
        time_s = data.get("time_s")
        if time_s is None:
            time_s = data.get("time")

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
            # Consistent fused data for Tasks 5 and 6
            "fused_pos": pos_ned,
            "fused_vel": vel_ned,
            "quat_log": quat,
        }
        save_mat(npz_path.with_suffix(".mat"), mat_out)

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
    task7_dir = results_dir / "task7" / tag
    with open(log_path, "a") as log:
        log.write("\nTASK 7: Evaluate residuals\n")
        msg = "Running Task 7 evaluation ..."
        logger.info(msg)
        log.write(msg + "\n")
        buf = io.StringIO()
        with redirect_stdout(buf):
            try:
                run_evaluation_npz(str(npz_path), str(task7_dir), tag)
            except Exception as e:  # pragma: no cover - graceful failure
                print(f"Task 7 failed: {e}")
        output = buf.getvalue()
        print(output, end="")
        log.write(output)

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

    print("TRIAD processing complete for X001")


if __name__ == "__main__":
    main()

