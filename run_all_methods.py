import argparse
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="Run GNSS_IMU_Fusion with multiple methods")
    args = parser.parse_args()

    cases = [
        ("IMU_X001.dat", "GNSS_X001.csv", "clean"),
        ("IMU_X002.dat", "GNSS_X002.csv", "noise"),
        ("IMU_X003.dat", "GNSS_X003.csv", "bias"),
    ]

    methods = ["TRIAD", "Davenport", "SVD", "ALL"]
    os.makedirs("logs", exist_ok=True)
    for imu, gnss, tag in cases:
        for m in methods:
            log_name = os.path.join("logs", f"{tag}_{m}.log")
            cmd = [
                sys.executable,
                "GNSS_IMU_Fusion.py",
                "--method",
                m,
                "--imu-file",
                imu,
                "--gnss-file",
                gnss,
            ]
            print(f"Running {tag} with method {m}...")
            with open(log_name, "w") as f:
                ret = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
            if ret.returncode != 0:
                print(f"{tag} {m} failed", file=sys.stderr)
                sys.exit(ret.returncode)

if __name__ == "__main__":
    main()
