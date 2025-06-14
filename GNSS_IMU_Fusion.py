#!/usr/bin/env python3
"""Command line entry point for IMU/GNSS fusion demo."""
import argparse
import numpy as np

from imu_fusion.data import load_gnss_csv, load_imu_dat



def main() -> None:
    parser = argparse.ArgumentParser(description="IMU/GNSS fusion demo")
    parser.add_argument("--all", action="store_true", help="run all dataset combinations")
    parser.add_argument("--gnss-file", default="GNSS_X001.csv", help="GNSS CSV file")
    parser.add_argument("--imu-file", default="IMU_X001.dat", help="IMU dat file")
    parser.add_argument(
        "--init-method",
        choices=["TRIAD", "Davenport", "SVD"],
        default="Davenport",
        help="Attitude initialization method",
    )
    args = parser.parse_args()




if __name__ == "__main__":
    main()
