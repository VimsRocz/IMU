#!/usr/bin/env python3
"""Command line entry point for IMU/GNSS fusion demo."""
import argparse
import numpy as np

from imu_fusion.data import load_gnss_csv, load_imu_dat
from imu_fusion.attitude import compute_C_ECEF_to_NED, rot_to_quaternion
from imu_fusion.kalman import simple_kalman
from imu_fusion.plotting import plot_ned_positions


def main() -> None:
    parser = argparse.ArgumentParser(description="IMU/GNSS fusion demo")
    parser.add_argument("--gnss-file", default="GNSS_X001.csv", help="GNSS CSV file")
    parser.add_argument("--imu-file", default="IMU_X001.dat", help="IMU dat file")
    args = parser.parse_args()

    gnss = load_gnss_csv(args.gnss_file)
    imu = load_imu_dat(args.imu_file)

    lat = float(gnss.iloc[0].get("Latitude_deg", 0.0))
    lon = float(gnss.iloc[0].get("Longitude_deg", 0.0))
    C = compute_C_ECEF_to_NED(np.deg2rad(lat), np.deg2rad(lon))
    q = rot_to_quaternion(C.T)
    print("Initial quaternion (body to NED):", q)

    if {'X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'} <= set(gnss.columns):
        z = gnss[['X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m']].values
    else:
        z = np.zeros((len(gnss), 3))
    F = np.eye(3)
    H = np.eye(3)
    Q = np.eye(3) * 0.01
    R = np.eye(3) * 0.1
    xs = simple_kalman(z, F, H, Q, R, z[0])
    plot_ned_positions(range(len(xs)), xs, "KF", "fusion_output.pdf")
    print("Kalman filter output written to fusion_output.pdf")


if __name__ == "__main__":
    main()
