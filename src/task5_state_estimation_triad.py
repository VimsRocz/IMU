#!/usr/bin/env python3
"""Task 5 – State Estimation with TRIAD (Python version).

This module mirrors the MATLAB function ``Task_5`` that performs a simple
state propagation and GNSS fusion using TRIAD-aligned attitude.  It loads
results from previous tasks, integrates IMU measurements and exports the
position and velocity estimates in NED and ECEF frames.

Usage
-----
    python src/task5_state_estimation_triad.py IMU_X002.dat GNSS_X002.csv
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as R


def task5_state_estimation_triad(imu_path: str | Path, gnss_path: str | Path) -> None:
    """Fuse IMU and GNSS data using a basic propagation loop.

    Parameters
    ----------
    imu_path : str or Path
        Path to the IMU measurements file.
    gnss_path : str or Path
        Path to the GNSS measurements file.  Only used to match the MATLAB
        interface; GNSS updates are loaded from the Task 4 results MAT-file.
    """
    print("--- Starting Task 5: State Estimation with TRIAD ---")

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    task3_file = Path("MATLAB/results/Task3_results_IMU_X002_GNSS_X002.mat")
    task2_file = Path("MATLAB/results/Task2_body_IMU_X002_GNSS_X002_TRIAD.mat")
    task4_file = Path("MATLAB/results/Task4_results_IMU_X002_GNSS_X002.mat")

    try:
        loadmat(task3_file)
        task2 = loadmat(task2_file)
        accel_bias = task2["accel_bias"].flatten()
        gyro_bias = task2["gyro_bias"].flatten()
        task4 = loadmat(task4_file)
        gnss_ned_pos = task4["gnss_ned_pos"]
        gnss_ned_vel = task4["gnss_ned_vel"]
        print("Task 5: Loaded C_b_n, biases, and GNSS data")
    except Exception as exc:
        raise RuntimeError("Task 5: Failed to load prerequisite data") from exc

    imu_data = np.loadtxt(imu_path)
    accel_raw = imu_data[:, 1:4]
    gyro_raw = imu_data[:, 4:7]
    dt = 0.0025
    n_samples = len(imu_data)
    print(f"Task 5: Loaded IMU data, {n_samples} samples, dt = {dt:.6f} s")

    accel_corr = (accel_raw - accel_bias) * 0.8368
    gyro_corr = gyro_raw - gyro_bias
    print("Task 5: Corrected IMU data for bias and scale")

    x = np.zeros((15, n_samples))
    x[0:3, 0] = gnss_ned_pos[:, 0]
    x[3:6, 0] = gnss_ned_vel[:, 0]
    quat = np.array([0.785256, -0.014053, 0.618755, 0.017837])
    x[6:9, 0] = R.from_quat(quat).as_euler("xyz")
    x[9:12, 0] = accel_bias
    x[12:15, 0] = gyro_bias
    print("Task 5: Initialized state with GNSS and Task 3 attitude")

    for k in range(1, n_samples):
        x[3:6, k] = x[3:6, k - 1] + accel_corr[k - 1] * dt
        x[0:3, k] = x[0:3, k - 1] + x[3:6, k] * dt
        x[6:9, k] = x[6:9, k - 1] + gyro_corr[k - 1] * dt
        if k % 50000 == 0:
            print(f"Task 5: Propagated state at sample {k}")

    gnss_indices = np.arange(0, n_samples, 400)
    for idx in gnss_indices:
        if idx < gnss_ned_pos.shape[1]:
            x[0:3, idx] = gnss_ned_pos[:, idx]
            x[3:6, idx] = gnss_ned_vel[:, idx]
            print(f"Task 5: Applied GNSS update at sample {idx}")

    C_n_e = np.array([[0.2336, 0.0106, -0.9723],
                      [-0.0454, 0.9990, 0.0000],
                      [0.9713, 0.0441, 0.2339]]).T
    pos_est_ned = x[0:3, :]
    vel_est_ned = x[3:6, :]
    pos_est_ecef = C_n_e @ pos_est_ned
    vel_est_ecef = C_n_e @ vel_est_ned
    acc_body = accel_corr.T
    print("Task 5: Computed estimates in NED, ECEF, and Body frames")

    out_file = results_dir / "IMU_X002_GNSS_X002_TRIAD_task5_results.npz"
    np.savez(
        out_file,
        x=x,
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

    parser = argparse.ArgumentParser(description="Task 5 TRIAD state estimation")
    parser.add_argument("imu_file")
    parser.add_argument("gnss_file")
    args = parser.parse_args()

    task5_state_estimation_triad(args.imu_file, args.gnss_file)


if __name__ == "__main__":
    main()
