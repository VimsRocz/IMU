#!/usr/bin/env python3
"""Command line entry point for IMU/GNSS fusion demo."""
import argparse
import numpy as np

from imu_fusion.data import load_gnss_csv, load_imu_dat
from imu_fusion.attitude import (
    compute_C_ECEF_to_NED,
    rot_to_quaternion,
    triad,
    davenport_q_method,
    svd_method,
)
from imu_fusion.kalman import kalman_with_residuals
from imu_fusion.plotting import plot_ned_positions, plot_residuals, plot_attitude


def main() -> None:
    parser = argparse.ArgumentParser(description="IMU/GNSS fusion demo")
    parser.add_argument("--gnss-file", default="GNSS_X001.csv", help="GNSS CSV file")
    parser.add_argument("--imu-file", default="IMU_X001.dat", help="IMU dat file")
    parser.add_argument(
        "--init-method",
        choices=["TRIAD", "Davenport", "SVD"],
        default="Davenport",
        help="Attitude initialization method",
    )
    args = parser.parse_args()

    gnss = load_gnss_csv(args.gnss_file)
    imu = load_imu_dat(args.imu_file)
    method = args.init_method

    lat = float(gnss.iloc[0].get("Latitude_deg", 0.0))
    lon = float(gnss.iloc[0].get("Longitude_deg", 0.0))
    C = compute_C_ECEF_to_NED(np.deg2rad(lat), np.deg2rad(lon))

    g_ned = np.array([0.0, 0.0, 9.81])
    omega_e = 7.2921159e-5
    omega_ned = np.array([omega_e * np.cos(np.deg2rad(lat)), 0.0, -omega_e * np.sin(np.deg2rad(lat))])

    dt_imu = 1.0 / 400.0
    acc_body = imu[:, 5:8] / dt_imu
    gyro_body = imu[:, 2:5] / dt_imu
    N_static = min(4000, len(acc_body))
    g_body = -np.mean(acc_body[:N_static], axis=0)
    omega_body = np.mean(gyro_body[:N_static], axis=0)

    if method == "TRIAD":
        R = triad(g_body, omega_body, g_ned, omega_ned)
    elif method == "SVD":
        R = svd_method(g_body, omega_body, g_ned, omega_ned)
    else:
        R = davenport_q_method(g_body, omega_body, g_ned, omega_ned)
    q = rot_to_quaternion(R)
    print(f"Initial quaternion ({method}):", q)

    if {"X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m", "VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"} <= set(gnss.columns):
        z = gnss[[
            "X_ECEF_m",
            "Y_ECEF_m",
            "Z_ECEF_m",
            "VX_ECEF_mps",
            "VY_ECEF_mps",
            "VZ_ECEF_mps",
        ]].values
    else:
        z = np.zeros((len(gnss), 6))

    times = gnss["Posix_Time"].values
    dt = float(np.mean(np.diff(times))) if len(times) > 1 else 1.0

    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    H = np.eye(6)
    Q = np.eye(6) * 0.01
    R = np.eye(6) * 0.1
    xs, residuals = kalman_with_residuals(z, F, H, Q, R, z[0])

    time_rel = times - times[0]
    plot_ned_positions(
        time_rel,
        xs[:, :3],
        "KF",
        "Kalman Filter Estimated Position",
        "positions.pdf",
    )
    plot_residuals(time_rel, residuals, "Kalman Filter Residuals", "residuals.pdf")

    yaw = np.degrees(np.arctan2(z[:, 4], z[:, 3]))
    pitch = np.zeros_like(yaw)
    roll = np.zeros_like(yaw)
    plot_attitude(time_rel, yaw, pitch, roll, "Attitude Angles", "attitude.pdf")

    print("Plots written: positions.pdf, residuals.pdf, attitude.pdf")


if __name__ == "__main__":
    main()
