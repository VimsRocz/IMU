#!/usr/bin/env python3
"""Run every (IMU, GNSS, method) combo, drop artefacts in results/"""
import subprocess, os, pathlib, itertools

cases   = [("IMU_X001.dat", "GNSS_X001.csv"),
           ("IMU_X002.dat", "GNSS_X002.csv")]
methods = ["TRIAD", "SVD", "Davenport"]

os.makedirs("results", exist_ok=True)

for (imu, gnss), m in itertools.product(cases, methods):
    tag = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{m}"
    log = open(f"results/{tag}.log", "w")
    print(f"\u25B6 {tag}")
    subprocess.run([
        "python",
        "GNSS_IMU_Fusion.py",
        "--imu-file",
        imu,
        "--gnss-file",
        gnss,
        "--method",
        m,
        "--no-plots",
    ], stdout=log, stderr=subprocess.STDOUT, check=True)
    log.close()
