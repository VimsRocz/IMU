#!/usr/bin/env python3
"""Task 5 – Sensor Fusion with Kalman Filter (Python stub).

This module replicates the updated MATLAB ``Task_5`` function.  It loads
pre-computed rotation matrices and GNSS data, runs a very basic Kalman
filter on IMU measurements and stores the state history as well as NED
and ECEF estimates.  The implementation mirrors the MATLAB code but omits
detailed dynamics modelling.

Usage
-----
    python task5_sensor_fusion_kf.py IMU_X002.dat GNSS_X002.csv
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
from scipy.io import loadmat


# ---------------------------------------------------------------------------
# helper functions
# ---------------------------------------------------------------------------

def task5_sensor_fusion_kf(imu_path: str | Path, gnss_path: str | Path) -> None:
    """Run the simplified 15-state Kalman filter.

    Parameters
    ----------
    imu_path : str or Path
        Path to the IMU data file.
    gnss_path : str or Path
        Unused placeholder for parity with MATLAB.  GNSS updates are
        loaded from the Task 4 results MAT-file.
    """

    print("--- Starting Task 5: Sensor Fusion with Kalman Filter ---")

    # Load prerequisite data from MATLAB results
    task3_file = Path("MATLAB/results/Task3_results_IMU_X002_GNSS_X002.mat")
    task4_file = Path("MATLAB/results/Task4_results_IMU_X002_GNSS_X002.mat")

    try:
        task3 = loadmat(task3_file)
        task4 = loadmat(task4_file)
        c_b_n = task3.get("C_b_n")
        gnss_ned_pos = task4.get("gnss_ned_pos")
        gnss_ned_vel = task4.get("gnss_ned_vel")
        if gnss_ned_pos is None:
            # Fallback variable names
            gnss_ned_pos = task4.get("GNSS_NED_POS")
            gnss_ned_vel = task4.get("GNSS_NED_VEL")
        if c_b_n is None or gnss_ned_pos is None:
            raise ValueError("Required variables missing from MAT-files")
        print("Task 5: Loaded C_b_n and GNSS data")
    except Exception as exc:
        raise RuntimeError("Task 5: Failed to load prerequisite data") from exc

    # Compute NED to ECEF rotation matrix (from Task 1 constants)
    lat = -31.871173 * np.pi / 180.0
    lon = 133.455811 * np.pi / 180.0
    c_n_e = np.array(
        [
            [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
            [-np.sin(lon), np.cos(lon), 0.0],
            [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)],
        ]
    )
    print("Task 5: Computed C_n_e from latitude and longitude")

    # Load IMU data
    imu_path = Path(imu_path)
    imu_data = np.loadtxt(imu_path)
    accel_raw = imu_data[:, 1:4]
    gyro_raw = imu_data[:, 4:7]
    dt = 0.0025
    n_samples = accel_raw.shape[0]
    print(f"Task 5: Loaded IMU data, {n_samples} samples, dt = {dt:.6f} s")

    # Initialise Kalman filter state
    x = np.zeros(15)
    x[0:3] = gnss_ned_pos[:, 0]
    x[3:6] = gnss_ned_vel[:, 0]
    x[6:10] = np.array([0.785256, -0.014053, 0.618755, 0.017837])
    x[10:13] = np.array([0.577573, -6.836713, 0.910290])
    x[13:15] = np.array([-0.000023, 0.000069])
    p = np.eye(15) * 0.1
    x_log = np.zeros((15, n_samples))
    x_log[:, 0] = x
    print("Task 5: Initialized 15-state Kalman filter")

    # Propagation loop
    for k in range(1, n_samples):
        accel_corr = accel_raw[k - 1] - x[10:13]
        gyro_corr = gyro_raw[k - 1] - np.array([x[13], 0.0, x[14]])
        x[3:6] += accel_corr * dt
        x[0:3] += x[3:6] * dt
        q = x[6:10]
        omega = gyro_corr
        omega_mat = np.array(
            [
                [0.0, -omega[0], -omega[1], -omega[2]],
                [omega[0], 0.0, omega[2], -omega[1]],
                [omega[1], -omega[2], 0.0, omega[0]],
                [omega[2], omega[1], -omega[0], 0.0],
            ]
        )
        q_dot = 0.5 * omega_mat @ q
        x[6:10] += q_dot * dt
        x[6:10] /= np.linalg.norm(x[6:10])
        x_log[:, k] = x
        if (k + 1) % 50000 == 0:
            print(f"Task 5: Propagated state at sample {k+1}")

    # GNSS updates every 400 samples
    gnss_indices = np.arange(0, n_samples, 400)
    h = np.hstack([np.eye(6), np.zeros((6, 9))])
    r = np.eye(6) * 0.01
    for idx in gnss_indices:
        if idx < gnss_ned_pos.shape[1]:
            z = np.hstack([gnss_ned_pos[:, idx], gnss_ned_vel[:, idx]])
            y = z - h @ x
            s = h @ p @ h.T + r
            k_gain = p @ h.T @ np.linalg.inv(s)
            x += k_gain @ y
            x[6:10] /= np.linalg.norm(x[6:10])
            p = (np.eye(15) - k_gain @ h) @ p
            x_log[:, idx] = x
            print(f"Task 5: Applied GNSS update at sample {idx+1}")

    # Transform to all frames
    pos_est_ned = x_log[0:3, :]
    vel_est_ned = x_log[3:6, :]
    pos_est_ecef = c_n_e @ pos_est_ned
    vel_est_ecef = c_n_e @ vel_est_ned
    acc_body = accel_raw.T
    print("Task 5: Computed estimates in NED, ECEF, and Body frames")

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)
    out_file = results_dir / "IMU_X002_GNSS_X002_TRIAD_task5_results.npz"
    np.savez(
        out_file,
        x_log=x_log,
        pos_est_ecef=pos_est_ecef,
        vel_est_ecef=vel_est_ecef,
        pos_est_ned=pos_est_ned,
        vel_est_ned=vel_est_ned,
        acc_body=acc_body,
    )
    print(f"Task 5: Results saved to {out_file}")
    print("Task 5: Completed successfully")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Task 5 Kalman filter stub")
    parser.add_argument("imu_file")
    parser.add_argument("gnss_file")
    args = parser.parse_args()

    task5_sensor_fusion_kf(args.imu_file, args.gnss_file)


if __name__ == "__main__":
    main()
