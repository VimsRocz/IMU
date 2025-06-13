#!/usr/bin/env python3
"""Command line entry point for IMU/GNSS fusion demo."""
import argparse
import numpy as np

from imu_fusion.data import load_gnss_csv, load_imu_dat
from imu_fusion.attitude import compute_C_ECEF_to_NED, rot_to_quaternion
from imu_fusion.plotting import plot_ned_positions, plot_residuals, plot_attitude
from filterpy.kalman import KalmanFilter


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

    # Kalman filter setup. The state vector contains ECEF position and velocity.
    kf = KalmanFilter(dim_x=6, dim_z=6, dim_u=3)
    kf.x = z[0]
    kf.P = np.eye(6)
    kf.H = np.eye(6)
    kf.Q = np.eye(6) * 0.01
    kf.R = np.eye(6) * 0.1

    dt_imu = float(imu[1, 1] - imu[0, 1])
    start_time = times[0]
    gnss_idx = 1
    next_gnss_time = times[gnss_idx] if gnss_idx < len(times) else float("inf")

    xs = [kf.x.copy()]
    residuals = [np.zeros(6)]

    for i in range(1, len(imu)):
        # Propagate with IMU acceleration at the IMU rate
        dt = dt_imu
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt
        B = np.zeros((6, 3))
        B[0:3, :] = 0.5 * dt * dt * np.eye(3)
        B[3:6, :] = dt * np.eye(3)
        kf.F = F
        kf.B = B
        acc = imu[i, 5:8] / dt
        kf.predict(u=acc)

        t_abs = start_time + i * dt
        while t_abs >= next_gnss_time and gnss_idx < len(z):
            residual = z[gnss_idx] - (kf.H @ kf.x)
            kf.update(z[gnss_idx])
            xs.append(kf.x.copy())
            residuals.append(residual)
            gnss_idx += 1
            next_gnss_time = (
                times[gnss_idx] if gnss_idx < len(times) else float("inf")
            )

        if gnss_idx >= len(times):
            break

    xs = np.array(xs)
    residuals = np.array(residuals)

    plot_times = times[: len(xs)]
    time_rel = plot_times - plot_times[0]
    plot_ned_positions(
        time_rel,
        xs[:, :3],
        "KF",
        "Kalman Filter Estimated Position",
        "positions.pdf",
    )
    plot_residuals(time_rel, residuals, "Kalman Filter Residuals", "residuals.pdf")

    yaw = np.degrees(np.arctan2(z[: len(xs), 4], z[: len(xs), 3]))
    pitch = np.zeros_like(yaw)
    roll = np.zeros_like(yaw)
    plot_attitude(time_rel, yaw, pitch, roll, "Attitude Angles", "attitude.pdf")

    print("Plots written: positions.pdf, residuals.pdf, attitude.pdf")


if __name__ == "__main__":
    main()
