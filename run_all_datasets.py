#!/usr/bin/env python3
"""
Run GNSS_IMU_Fusion.py on all required data-set pairs in one shot.

Adds a `[SUMMARY]` line to the console for each run – GNSS_IMU_Fusion
already emits this so we just relay the output.
"""

import subprocess
from pathlib import Path

# ----------------------------------------------------------------------
# Edit this table whenever you have new files to process.
DATASETS = [
    # (IMU file,         GNSS file)
    ("IMU_X002.dat",     "GNSS_X002.csv"),  # noisy-noisy
    ("IMU_X001.dat",     "GNSS_X001.csv"),  # clean-clean
    ("IMU_X003.dat",     "GNSS_X002.csv"),  # bias-noisy
]
# ----------------------------------------------------------------------

def run_pair(imu_file: str, gnss_file: str) -> None:
    """Call GNSS_IMU_Fusion.py for a single IMU–GNSS pair."""
    cmd = [
        "python", "GNSS_IMU_Fusion.py",
        "--imu-file", imu_file,
        "--gnss-file", gnss_file,
    ]
    print("\n>>>", " ".join(cmd))
    subprocess.run(cmd, check=True)   # raises if the script crashes


def main() -> None:
    # Verify that the user is in the repo root (so the files resolve)
    here = Path.cwd()
    for imu, gnss in DATASETS:
        if not Path(imu).is_file():
            raise FileNotFoundError(f"{imu} not found in {here}")
        if not Path(gnss).is_file():
            raise FileNotFoundError(f"{gnss} not found in {here}")

    for imu, gnss in DATASETS:
        run_pair(imu, gnss)


if __name__ == "__main__":
    main()
