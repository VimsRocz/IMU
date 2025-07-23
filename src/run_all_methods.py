#!/usr/bin/env python3
"""Run every (IMU, GNSS, method) combo and log the output.

The script processes each IMU/GNSS pair with all selected methods and writes
the console output to ``results/<IMU>_<GNSS>_<method>.log``.  By default the
bundled ``IMU_X`` data sets are used, but a YAML configuration file can
override the data files and the list of methods.

Example ``config.yml``::

    datasets:
      - imu: IMU_X001.dat
        gnss: GNSS_X001.csv
      - imu: IMU_X002.dat
        gnss: GNSS_X002.csv
    methods: [TRIAD, Davenport, SVD]

Run the script with ``--config config.yml`` to process those files.  The
``--datasets`` and ``--method`` options mirror ``run_all_datasets.py`` and
restrict the processed data sets and attitude initialisation method.
"""

import argparse
import itertools
import os
import pathlib
import subprocess
import sys
from typing import Iterable, Tuple
import logging
import re
import time
import pandas as pd
from tabulate import tabulate
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import save_mat
from contextlib import redirect_stdout
import io
from evaluate_filter_results import run_evaluation_npz

from utils import compute_C_ECEF_to_NED

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

try:
    import yaml
except ModuleNotFoundError:  # allow running without PyYAML installed
    yaml = None

DEFAULT_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
    ("IMU_X002.dat", "GNSS_X002.csv"),
    ("IMU_X003.dat", "GNSS_X002.csv"),  # dataset X003 shares GNSS_X002
]

DEFAULT_METHODS = ["TRIAD", "SVD", "Davenport"]

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def load_config(path: str):
    """Return (datasets, methods) from a YAML config file."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to use --config")
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    datasets = [
        (item["imu"], item["gnss"]) for item in data.get("datasets", [])
    ] or list(DEFAULT_DATASETS)
    methods = data.get("methods", DEFAULT_METHODS)
    return datasets, methods


def compute_C_NED_to_ECEF(lat: float, lon: float) -> np.ndarray:
    """Return rotation matrix from NED to ECEF frame."""
    return compute_C_ECEF_to_NED(lat, lon).T


def run_case(cmd, log_path):
    """Run a single fusion command and log output live to console and file."""
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        summary_lines = []
        for line in proc.stdout:
            print(line, end="")
            log.write(line)
            m = SUMMARY_RE.search(line)
            if m:
                summary_lines.append(m.group(1))
        proc.wait()
        return proc.returncode, summary_lines


def main(argv=None):
    os.makedirs("results", exist_ok=True)
    logger.info("Ensured 'results/' directory exists.")
    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with multiple datasets and methods",
    )
    parser.add_argument(
        "--config",
        help="YAML file specifying datasets and methods",
    )
    parser.add_argument(
        "--datasets",
        default="ALL",
        help="Comma separated dataset IDs (e.g. X001,X002) or ALL",
    )
    parser.add_argument(
        "--method",
        choices=["TRIAD", "SVD", "Davenport", "ALL"],
        default="ALL",
        help="Run only one attitude initialisation method",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation for faster execution",
    )
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
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose debug output",
    )
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

    if args.config:
        cases, methods = load_config(args.config)
    else:
        cases, methods = list(DEFAULT_DATASETS), list(DEFAULT_METHODS)

    if args.datasets.upper() != "ALL":
        ids = {d.strip() for d in args.datasets.split(",")}
        cases = [p for p in cases if pathlib.Path(p[0]).stem.split("_")[1] in ids]

    if args.method.upper() == "ALL":
        method_list = methods
    else:
        method_list = [args.method]

    logger.debug(f"Datasets: {cases}")
    logger.debug(f"Methods: {method_list}")

    results = []
    for (imu, gnss), m in itertools.product(cases, method_list):
        tag = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{m}"
        log_path = pathlib.Path("results") / f"{tag}.log"
        print(f"\u25b6 {tag}")
        if logger.isEnabledFor(logging.DEBUG):
            try:
                gnss_preview = np.loadtxt(
                    ROOT / gnss, delimiter=",", skiprows=1, max_rows=1
                )
                imu_preview = np.loadtxt(ROOT / imu, max_rows=1)
                logger.debug(
                    f"GNSS preview: shape {gnss_preview.shape}, first row: {gnss_preview}"
                )
                logger.debug(
                    f"IMU preview: shape {imu_preview.shape}, first row: {imu_preview}"
                )
            except Exception as e:
                logger.warning(f"Failed data preview for {imu} or {gnss}: {e}")
        cmd = [
            sys.executable,
            str(HERE / "GNSS_IMU_Fusion.py"),
            "--imu-file",
            str(ROOT / imu),
            "--gnss-file",
            str(ROOT / gnss),
            "--method",
            m,
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
                    "method": kv.get("method", m),
                    "rmse_pos": float(kv.get("rmse_pos", "nan").replace("m", "")),
                    "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                    "rms_resid_pos": float(
                        kv.get("rms_resid_pos", "nan").replace("m", "")
                    ),
                    "max_resid_pos": float(
                        kv.get("max_resid_pos", "nan").replace("m", "")
                    ),
                    "rms_resid_vel": float(
                        kv.get("rms_resid_vel", "nan").replace("m", "")
                    ),
                    "max_resid_vel": float(
                        kv.get("max_resid_vel", "nan").replace("m", "")
                    ),
                    "runtime": runtime,
                }
            )
        # ------------------------------------------------------------------
        # Convert NPZ output to a MATLAB file with explicit frame variables
        # ------------------------------------------------------------------
        npz_path = pathlib.Path("results") / f"{tag}_kf_output.npz"
        if npz_path.exists():
            data = np.load(npz_path, allow_pickle=True)
            logger.debug(f"Loaded output {npz_path} with keys: {list(data.keys())}")
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
                    f"Output time range: {time_s[0] if time_s is not None else 'N/A'}"
                    f" to {time_s[-1] if time_s is not None else 'N/A'} s"
                )
                logger.debug(
                    f"Position shape: {pos_ned.shape if pos_ned is not None else 'None'}"
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
                "method_name": m,
                # Consistent fused data for Tasks 5 and 6
                "fused_pos": pos_ned,
                "fused_vel": vel_ned,
                "quat_log": quat,
            }
            save_mat(npz_path.with_suffix(".mat"), mat_out)

            logger.info(
                "Subtask 6.8.2: Plotted %s position North: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 0],
                pos_ned[-1, 0],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s position East: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 1],
                pos_ned[-1, 1],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s position Down: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 2],
                pos_ned[-1, 2],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity North: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 0],
                vel_ned[-1, 0],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity East: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 1],
                vel_ned[-1, 1],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity Down: First = %.4f, Last = %.4f",
                m,
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

            # ----------------------------
            # Task 7: Evaluation
            # ----------------------------
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

    # --- nicely formatted summary table --------------------------------------
    if results:
        key_order = {m: i for i, m in enumerate(methods)}
        results.sort(key=lambda r: (r["dataset"], key_order.get(r["method"], 0)))
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
