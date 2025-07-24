#!/usr/bin/env python3
"""Run all datasets with the TRIAD initialisation and evaluate Tasks 6/7.

This helper mirrors ``run_all_methods.py`` but restricts the attitude
initialisation to the TRIAD method. Each IMU/GNSS pair is processed in
sequence and, when ``STATE_X*.txt`` ground truth logs are available, the
Task 6 overlay plots and Task 7 residual analysis are generated under
``results/``.

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
from typing import List, Dict

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from tabulate import tabulate

from evaluate_filter_results import run_evaluation_npz
from run_all_methods import (
    DEFAULT_DATASETS,
    load_config,
    run_case,
    compute_C_NED_to_ECEF,
)
from utils import save_mat

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def main(argv: List[str] | None = None) -> None:
    os.makedirs("results", exist_ok=True)
    logger.info("Ensured 'results/' directory exists.")

    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with the TRIAD method on all datasets",
    )
    parser.add_argument("--config", help="Optional YAML config overriding datasets")
    parser.add_argument("--no-plots", action="store_true", help="Skip Task 5 plots")
    parser.add_argument(
        "--show-measurements",
        action="store_true",
        help="Include IMU/GNSS measurements in Task 6 overlay plots",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")

    args = parser.parse_args(argv)

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    if args.config:
        cases, _ = load_config(args.config)
    else:
        cases = list(DEFAULT_DATASETS)

    method = "TRIAD"
    results: List[Dict[str, float | str]] = []

    for imu, gnss in cases:
        tag = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{method}"
        log_path = pathlib.Path("results") / f"{tag}.log"
        print(f"\u25b6 {tag}")

        cmd = [
            sys.executable,
            str(HERE / "GNSS_IMU_Fusion.py"),
            "--imu-file",
            str(ROOT / imu),
            "--gnss-file",
            str(ROOT / gnss),
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

        npz_path = pathlib.Path("results") / f"{tag}_kf_output.npz"
        if npz_path.exists():
            data = np.load(npz_path, allow_pickle=True)
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

            ref_lat = data.get("ref_lat_rad")
            if ref_lat is None:
                ref_lat = data.get("ref_lat")
            ref_lat = float(np.squeeze(ref_lat if ref_lat is not None else 0.0))

            ref_lon = data.get("ref_lon_rad")
            if ref_lon is None:
                ref_lon = data.get("ref_lon")
            ref_lon = float(np.squeeze(ref_lon if ref_lon is not None else 0.0))

            ref_r0 = data.get("ref_r0_m")
            if ref_r0 is None:
                ref_r0 = data.get("ref_r0")
            if ref_r0 is None:
                ref_r0 = [0, 0, 0]
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
                "fused_pos": pos_ned,
                "fused_vel": vel_ned,
                "quat_log": quat,
            }
            save_mat(npz_path.with_suffix(".mat"), mat_out)

            truth_file = ROOT / "STATE_X001.txt"
            if truth_file.exists():
                overlay_cmd = [
                    sys.executable,
                    str(HERE / "task6_plot_truth.py"),
                    "--est-file",
                    str(npz_path.with_suffix(".mat")),
                    "--truth-file",
                    str(truth_file),
                    "--output",
                    "results",
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

            task7_dir = pathlib.Path("results") / "task7" / tag
            with open(log_path, "a") as log:
                log.write("\nTASK 7: Evaluate residuals\n")
                msg = "Running Task 7 evaluation ..."
                logger.info(msg)
                log.write(msg + "\n")
                buf = io.StringIO()
                with redirect_stdout(buf):
                    try:
                        run_evaluation_npz(str(npz_path), str(task7_dir), tag)
                    except Exception as e:
                        print(f"Task 7 failed: {e}")
                output = buf.getvalue()
                print(output, end="")
                log.write(output)

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
        df.to_csv(pathlib.Path("results") / "summary.csv", index=False)


if __name__ == "__main__":
    main()
