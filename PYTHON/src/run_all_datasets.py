#!/usr/bin/env python3
"""
Batch-runner for all IMU/GNSS sets and all attitude-initialisation methods.
Writes one log file per run in ./logs and prints a short SUMMARY line
that `summarise_runs.py` can aggregate later.
"""

import pathlib
from pathlib import Path as _Path
import sys as _sys
_SRC = _Path(__file__).resolve().parent
if str(_SRC) not in _sys.path:
    _sys.path.insert(0, str(_SRC))
REPO_ROOT = _SRC.parents[2]
import subprocess
import datetime
import sys
import argparse
import re
import time
import pandas as pd
import numpy as np
import yaml
import os
import logging
from utils import save_mat

from utils import ensure_dependencies, ecef_to_geodetic
from tabulate import tabulate
from tqdm import tqdm
# Overlay helper functions
from validate_with_truth import load_estimate, assemble_frames
from plot_overlay import plot_overlay

logging.basicConfig(level=logging.INFO, format="%(message)s")

ensure_dependencies()

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
from paths import ensure_results_dir as _ensure_results, truth_path as _truth_path_helper
SCRIPT = HERE / "GNSS_IMU_Fusion.py"
LOG_DIR = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)

DEFAULT_DATASETS = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
    ("IMU_X002.dat", "GNSS_X002.csv"),
    ("IMU_X003.dat", "GNSS_X002.csv"),  # dataset X003 shares GNSS_X002
]

DEFAULT_METHODS = ["TRIAD", "Davenport", "SVD"]

DATASETS = DEFAULT_DATASETS.copy()
METHODS = DEFAULT_METHODS.copy()

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def run_one(imu, gnss, method, verbose=False):
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = LOG_DIR / f"{imu}_{gnss}_{method}_{ts}.log"
    imu_path = pathlib.Path(imu)
    gnss_path = pathlib.Path(gnss)

    cmd = [
        sys.executable,
        SCRIPT,
        "--imu-file",
        str(imu_path),
        "--gnss-file",
        str(gnss_path),
        "--method",
        method,
    ]
    if verbose:
        cmd.append("--verbose")
    summary_lines = []
    with log.open("w") as fh:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        for line in proc.stdout:  # live-stream to console & file
            print(line, end="")
            fh.write(line)
            m = SUMMARY_RE.search(line)
            if m:
                summary_lines.append(m.group(1))
    if proc.wait() != 0:
        raise RuntimeError(f"{cmd} failed, see {log}")
    return summary_lines


def main():
    results_dir = _ensure_results()
    logging.info("Ensured '%s' directory exists.", results_dir)
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action="store_true", help="Print detailed debug info")
    parser.add_argument("--datasets", default="ALL",
                        help="Comma separated dataset IDs (e.g. X001,X002) or ALL")
    parser.add_argument(
        '--method',
        choices=['TRIAD', 'Davenport', 'SVD', 'ALL'],
        default='ALL',
    )
    parser.add_argument('--config', help='YAML configuration file')
    args = parser.parse_args()

    if args.config:
        with open(args.config) as fh:
            cfg = yaml.safe_load(fh) or {}
        global DATASETS, METHODS
        if 'datasets' in cfg:
            DATASETS = [(ds['imu'], ds['gnss']) for ds in cfg['datasets']]
        if 'methods' in cfg:
            METHODS = cfg['methods']

    if args.datasets.upper() == "ALL":
        datasets = DATASETS
    else:
        ids = {d.strip() for d in args.datasets.split(',')}
        datasets = [p for p in DATASETS if pathlib.Path(p[0]).stem.split('_')[1] in ids]

    if args.method.upper() == 'ALL':
        method_list = METHODS
    else:
        method_list = [args.method]
    cases = [(imu, gnss, m) for (imu, gnss) in datasets for m in method_list]
    fusion_results = []

    for imu, gnss, method in tqdm(cases, desc="All cases"):
        if args.verbose:
            # Debugging information for file pairing and timestamps
            print("==== DEBUG: File Pairing ====")
            print("IMU file:", imu)
            print("GNSS file:", gnss)
            gnss_df = pd.read_csv(gnss)
            imu_data = np.loadtxt(imu)
            print("GNSS shape:", gnss_df.shape)
            print("IMU shape:", imu_data.shape)
            print("GNSS time [start, end]:", gnss_df['Posix_Time'].iloc[0], gnss_df['Posix_Time'].iloc[-1])
            print("IMU time [start, end]:", imu_data[0, 0], imu_data[-1, 0])
            print("Any NaNs in GNSS?", gnss_df.isna().sum().sum())
            print("Any NaNs in IMU?", np.isnan(imu_data).sum())
            print("GNSS Head:\n", gnss_df.head())
            print("IMU Head:\n", imu_data[:5])
            print("============================")
        imu_path = pathlib.Path(imu)
        gnss_path = pathlib.Path(gnss)

        start = time.time()
        summaries = run_one(imu, gnss, method, verbose=args.verbose)
        runtime = time.time() - start
        for summary in summaries:
            kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
            fusion_results.append({
                "dataset"  : pathlib.Path(imu).stem.split("_")[1],
                "method"   : kv.get("method", method),
                "rmse_pos" : float(kv.get("rmse_pos", "nan").replace("m", "")),
                "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                "rms_resid_pos": float(kv.get("rms_resid_pos", "nan").replace("m", "")),
                "max_resid_pos": float(kv.get("max_resid_pos", "nan").replace("m", "")),
                "rms_resid_vel": float(kv.get("rms_resid_vel", "nan").replace("m", "")),
                "max_resid_vel": float(kv.get("max_resid_vel", "nan").replace("m", "")),
                "accel_bias": float(kv.get("accel_bias", "nan")),
                "gyro_bias": float(kv.get("gyro_bias", "nan")),
                "grav_mean_deg": float(kv.get("GravErrMean_deg", "nan")),
                "grav_max_deg": float(kv.get("GravErrMax_deg", "nan")),
                "earth_mean_deg": float(kv.get("EarthRateErrMean_deg", "nan")),
                "earth_max_deg": float(kv.get("EarthRateErrMax_deg", "nan")),
                "q0_w"      : float(kv.get("q0_w", "nan")),
                "q0_x"      : float(kv.get("q0_x", "nan")),
                "q0_y"      : float(kv.get("q0_y", "nan")),
                "q0_z"      : float(kv.get("q0_z", "nan")),
                "Pxx"       : float(kv.get("Pxx", "nan")),
                "Pyy"       : float(kv.get("Pyy", "nan")),
                "Pzz"       : float(kv.get("Pzz", "nan")),
                "ZUPT_count": int(kv.get("ZUPT_count", "0")),
                "runtime"  : runtime,
            })

        # --- Save standard MATLAB output ---------------------------------
        npz_path = results_dir / f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{method}_kf_output.npz"
        if npz_path.exists():
            data = np.load(npz_path, allow_pickle=True)
            mat_out = {
                "t": data.get("time_residuals"),
                "pos": data.get("fused_pos"),
                "vel": data.get("fused_vel"),
                "P": data.get("P_hist"),
                "quat": data.get("attitude_q"),
            }
            for key in data.files:
                if key not in ["time_residuals", "fused_pos", "fused_vel", "P_hist", "attitude_q"]:
                    mat_out[key] = data[key]
            save_mat(
                results_dir / f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{method}_kf_output.mat",
                mat_out,
            )

        truth_path = _truth_path_helper()
        est_mat = results_dir / f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{method}_kf_output.mat"
        if truth_path.exists():
            first = np.loadtxt(truth_path, comments="#", max_rows=1)
            r0 = first[2:5]
            lat_deg, lon_deg, _ = ecef_to_geodetic(*r0)

            vcmd = [
                sys.executable,
                str(HERE / "validate_with_truth.py"),
                "--est-file",
                str(est_mat),
                "--truth-file",
                str(truth_path),
                "--output",
                str(results_dir),
                "--ref-lat",
                str(lat_deg),
                "--ref-lon",
                str(lon_deg),
                "--ref-r0",
                str(r0[0]),
                str(r0[1]),
                str(r0[2]),
            ]
            subprocess.run(vcmd, check=True)
            try:
                truth_data = np.loadtxt(truth_path)
                t_truth = truth_data[:, 1]
                est = load_estimate(str(est_mat), times=t_truth)
                frames = assemble_frames(est, imu_path, gnss_path, truth_path)
                for frame_name, data in frames.items():
                    t_i, p_i, v_i, a_i = data["imu"]
                    t_g, p_g, v_g, a_g = data["gnss"]
                    t_f, p_f, v_f, a_f = data["fused"]
                    truth = data.get("truth")
                    if truth is not None:
                        t_t, p_t, v_t, a_t = truth
                    else:
                        t_t = p_t = v_t = a_t = None
                    plot_overlay(
                        frame_name,
                        method,
                        t_i,
                        p_i,
                        v_i,
                        a_i,
                        t_g,
                        p_g,
                        v_g,
                        a_g,
                        t_f,
                        p_f,
                        v_f,
                        a_f,
                        results_dir,
                        t_truth=t_t,
                        pos_truth=p_t,
                        vel_truth=v_t,
                        acc_truth=a_t,
                    )
            except Exception as e:
                print(f"Truth overlay failed: {e}")

    # --- nicely formatted summary table --------------------------------------
    key_order = {m: i for i, m in enumerate(["TRIAD", "Davenport", "SVD"])}
    fusion_results.sort(key=lambda r: (r["dataset"], key_order[r["method"]]))
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
            e["accel_bias"],
            e["gyro_bias"],
            e.get("grav_mean_deg", float('nan')),
            e.get("grav_max_deg", float('nan')),
            e.get("earth_mean_deg", float('nan')),
            e.get("earth_max_deg", float('nan')),
            e.get("q0_w", float('nan')),
            e.get("q0_x", float('nan')),
            e.get("q0_y", float('nan')),
            e.get("q0_z", float('nan')),
            e.get("Pxx", float('nan')),
            e.get("Pyy", float('nan')),
            e.get("Pzz", float('nan')),
            e.get("ZUPT_count", float('nan')),
            e["runtime"],
        ]
        for e in fusion_results
    ]
    print(tabulate(
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
            "AccelBiasNorm",
            "GyroBiasNorm",
            "GravErrMean [deg]",
            "GravErrMax [deg]",
            "EarthRateMean [deg]",
            "EarthRateMax [deg]",
            "q0_w",
            "q0_x",
            "q0_y",
            "q0_z",
            "Pxx",
            "Pyy",
            "Pzz",
            "ZUPTcnt",
            "Runtime [s]",
        ],
        floatfmt=".2f",
    ))

    # Optional CSV export for easier analysis
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
            "AccelBiasNorm",
            "GyroBiasNorm",
            "GravErrMean_deg",
            "GravErrMax_deg",
            "EarthRateErrMean_deg",
            "EarthRateErrMax_deg",
            "q0_w",
            "q0_x",
            "q0_y",
            "q0_z",
            "Pxx",
            "Pyy",
            "Pzz",
            "ZUPT_count",
            "Runtime_s",
        ],
    )
    df.to_csv(results_dir / "summary.csv", index=False)


if __name__ == "__main__":
    main()
