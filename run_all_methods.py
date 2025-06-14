import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="Run GNSS_IMU_Fusion with multiple methods")
    parser.add_argument("--imu-file", default="IMU_X001.dat", help="Path to IMU DAT file")
    parser.add_argument("--gnss-file", default="GNSS_X001.csv", help="Path to GNSS CSV file")
    args = parser.parse_args()

    methods = ["TRIAD", "Davenport", "SVD"]
    for m in methods:
        cmd = [sys.executable, "GNSS_IMU_Fusion.py", "--method", m,
               "--imu-file", args.imu_file, "--gnss-file", args.gnss_file]
        print(f"Running method {m}...")
        ret = subprocess.run(cmd)
        if ret.returncode != 0:
            print(f"Method {m} failed", file=sys.stderr)
            sys.exit(ret.returncode)

if __name__ == "__main__":
    main()
