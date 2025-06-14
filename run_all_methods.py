import argparse
import os
import subprocess
import sys
import pathlib


def main():
    parser = argparse.ArgumentParser(description="Run GNSS_IMU_Fusion with multiple methods")
    args = parser.parse_args()

    pairs = [
        ("IMU_X001.dat", "GNSS_X001.csv"),
        ("IMU_X002.dat", "GNSS_X002.csv"),
    ]
    if os.path.exists("GNSS_X003.csv"):
        pairs.append(("IMU_X003.dat", "GNSS_X003.csv"))
        pairs.append(("IMU_X002.dat", "GNSS_X003.csv"))

    methods = ["TRIAD", "Davenport", "SVD", "ALL"]
    os.makedirs("results", exist_ok=True)
    for imu, gnss in pairs:
        stem = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}"
        for m in methods:
            log_name = os.path.join("results", f"{stem}_{m}.log")
            cmd = [
                sys.executable,
                "GNSS_IMU_Fusion.py",
                "--method",
                m,
                "--imu-file",
                imu,
                "--gnss-file",
                gnss,
                "--output-dir",
                "results",
            ]
            print(f"Running {stem} with method {m}...")
            with open(log_name, "w") as f:
                ret = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
            if ret.returncode != 0:
                print(f"{stem} {m} failed", file=sys.stderr)
                sys.exit(ret.returncode)

if __name__ == "__main__":
    main()
