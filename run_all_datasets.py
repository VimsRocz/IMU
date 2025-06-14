#!/usr/bin/env python3
"""
Batch-runner for all IMU/GNSS sets and all attitude-initialisation methods.
Writes one log file per run in ./logs and prints a short SUMMARY line
that `summarise_runs.py` can aggregate later.
"""

import pathlib, subprocess, datetime, itertools, sys

HERE     = pathlib.Path(__file__).resolve().parent
SCRIPT   = HERE / "GNSS_IMU_Fusion.py"
LOG_DIR  = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)

DATASETS = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
    ("IMU_X002.dat", "GNSS_X002.csv"),
    ("IMU_X003.dat", "GNSS_X002.csv"),   # <- note the GNSS swap
]

METHODS  = ["TRIAD", "Davenport", "SVD", "ALL"]

def run_one(imu, gnss, method):
    ts    = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log   = LOG_DIR / f"{imu}_{gnss}_{method}_{ts}.log"
    cmd   = [
        sys.executable, SCRIPT,
        "--imu-file",  imu,
        "--gnss-file", gnss,
        "--method",    method
    ]
    print(f"\n─── Running {imu}/{gnss}  with  {method} ───")
    with log.open("w") as fh:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:                 # live-stream to console & file
            print(line, end="")
            fh.write(line)
    if proc.wait() != 0:
        raise RuntimeError(f"{cmd} failed, see {log}")

def main():
    here_files = {p.name for p in HERE.iterdir()}
    for imu, gnss in DATASETS:
        if imu not in here_files or gnss not in here_files:
            raise FileNotFoundError(f"Missing {imu} or {gnss} in {HERE}")
        for method in METHODS:
            run_one(imu, gnss, method)

if __name__ == "__main__":
    main()
