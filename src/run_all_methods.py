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

Run the script with ``--config config.yml`` to process those files.
"""

import argparse
import itertools
import os
import pathlib
import subprocess
import sys
from typing import Iterable, Tuple
import logging
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.io import savemat

from utils import compute_C_ECEF_to_NED

logging.basicConfig(level=logging.INFO, format="%(message)s")
os.makedirs('results', exist_ok=True)
logging.info("Ensured 'results/' directory exists.")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

try:
    import yaml
except ModuleNotFoundError:  # allow running without PyYAML installed
    yaml = None

DEFAULT_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X002.dat", "GNSS_X002.csv"),
]

DEFAULT_METHODS = ["TRIAD", "SVD", "Davenport"]


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
        for line in proc.stdout:
            print(line, end="")
            log.write(line)
        proc.wait()
        return proc.returncode


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with multiple datasets and methods",
    )
    parser.add_argument(
        "--config",
        help="YAML file specifying datasets and methods",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation for faster execution",
    )
    args = parser.parse_args(argv)

    if args.config:
        cases, methods = load_config(args.config)
    else:
        cases, methods = list(DEFAULT_DATASETS), list(DEFAULT_METHODS)

    for (imu, gnss), m in itertools.product(cases, methods):
        tag = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{m}"
        log_path = pathlib.Path("results") / f"{tag}.log"
        print(f"\u25B6 {tag}")
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
        ret = run_case(cmd, log_path)
        if ret != 0:
            raise subprocess.CalledProcessError(ret, cmd)
        # ------------------------------------------------------------------
        # Convert NPZ output to a MATLAB file with explicit frame variables
        # ------------------------------------------------------------------
        npz_path = pathlib.Path("results") / f"{tag}_kf_output.npz"
        if npz_path.exists():
            data = np.load(npz_path, allow_pickle=True)
            time_s = data.get("time")
            pos_ned = data.get("pos_ned")
            vel_ned = data.get("vel_ned")
            if pos_ned is None:
                pos_ned = data.get("fused_pos")
            if vel_ned is None:
                vel_ned = data.get("fused_vel")
            ref_lat = float(np.squeeze(data.get("ref_lat")))
            ref_lon = float(np.squeeze(data.get("ref_lon")))
            ref_r0 = np.asarray(data.get("ref_r0"))

            C_NED_ECEF = compute_C_NED_to_ECEF(ref_lat, ref_lon)
            pos_ecef = (C_NED_ECEF @ pos_ned.T).T + ref_r0
            vel_ecef = (C_NED_ECEF @ vel_ned.T).T

            quat = data.get("attitude_q")
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
            }
            savemat(npz_path.with_suffix(".mat"), mat_out)


if __name__ == "__main__":
    main()
