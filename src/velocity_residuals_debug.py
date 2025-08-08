"""Velocity residual diagnostics and filter tuning helper.

Usage:
    python velocity_residuals_debug.py --gnss-file GNSS.csv --imu-file IMU.dat \
        --fused-file estimate.npz --output-dir results

This script prints basic statistics to help debug high velocity residuals
gravity check and optionally plots the Z velocity component.  Figures are
stored in the specified output directory.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from validate_with_truth import load_estimate
from utils import zero_base_time



def main() -> None:
    parser = argparse.ArgumentParser(description="Velocity residual diagnostics")
    parser.add_argument("--gnss-file", required=True)
    parser.add_argument("--imu-file", required=True)
    parser.add_argument("--fused-file", required=True)
    parser.add_argument("--output-dir", default="results")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load GNSS data
    gnss = pd.read_csv(args.gnss_file)
    gnss_time = zero_base_time(gnss["Posix_Time"].to_numpy())
    gnss_vel_ecef = gnss[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].to_numpy()

    # Load fused estimate
    est = load_estimate(args.fused_file)
    fused_time = zero_base_time(est["time"])
    fused_vel_ecef = est["vel"]

    # Load IMU data and derive timestamps
    imu_raw = np.loadtxt(args.imu_file)
    dt_imu = 1.0 / 400.0
    imu_time = np.arange(len(imu_raw)) * dt_imu
    if imu_raw.shape[1] >= 10:
        imu_acc = imu_raw[:, 5:8] / dt_imu
    else:
        imu_acc = imu_raw[:, 2:5] / dt_imu

    # --- Section 1: Time stamps and initial values ---------------------------------
    print("First 10 time stamps in GNSS:", gnss_time[:10])
    print("First 10 time stamps in IMU:", imu_time[:10])
    print("Are time arrays equally spaced? GNSS dt:", np.diff(gnss_time[:10]))
    print("Are time arrays equally spaced? IMU dt:", np.diff(imu_time[:10]))
    print("First 5 GNSS ECEF velocities:", gnss_vel_ecef[:5])
    print("First 5 fused ECEF velocities:", fused_vel_ecef[:5])

    # Interpolate fused velocity to GNSS time for residuals
    fused_vel_interp = np.vstack(
        [np.interp(gnss_time, fused_time, fused_vel_ecef[:, i]) for i in range(3)]
    ).T

    # --- Section 2: Residual statistics --------------------------------------------
    velocity_residuals = fused_vel_interp - gnss_vel_ecef
    residual_norm = np.linalg.norm(velocity_residuals, axis=1)
    print("Velocity residuals mean (ECEF):", velocity_residuals.mean(axis=0))
    print("Velocity residuals std (ECEF):", velocity_residuals.std(axis=0))
    print(
        "Velocity residuals norm (mean/std):",
        residual_norm.mean(),
        residual_norm.std(),
    )

    # --- Section 3: Gravity compensation diagnostics --------------------------------
    gravity_ecef = np.array([0.0, 0.0, 9.81])
    print("Gravity vector used for compensation (ECEF):", gravity_ecef)
    acc_mean = imu_acc.mean(axis=0)
    print("Mean raw IMU acceleration (ECEF):", acc_mean)
    print("Mean after gravity compensation (ECEF):", acc_mean - gravity_ecef)

    # --- Section 4: Process/measurement noise matrices -----------------------------
    if est.get("Q") is not None:
        q = np.asarray(est["Q"])
        if q.ndim == 2 and q.shape[0] >= 6:
            print("Process noise matrix Q for velocity states:\n", q[3:6, 3:6])
    if est.get("R") is not None:
        r = np.asarray(est["R"])
        if r.ndim == 2 and r.shape[0] >= 6:
            print("Measurement noise matrix R for velocity states:\n", r[3:6, 3:6])

    # --- Section 5: Time alignment --------------------------------------------------
    max_offset = np.max(np.abs(imu_time[: len(gnss_time)] - gnss_time))
    print("Max time offset between IMU and GNSS samples:", max_offset)

    # --- Section 6: Diagnostic plot -------------------------------------------------
    plt.figure()
    plt.plot(gnss_time, gnss_vel_ecef[:, 2], label="GNSS Velocity Z [ECEF]")
    plt.plot(gnss_time, fused_vel_interp[:, 2], label="Fused Velocity Z [ECEF]")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity Z [m/s]")
    plt.legend()
    plt.title("Z Velocity: GNSS vs Fused")
    plt.grid(True)
    pdf = out_dir / "velocity_z_overlay.pdf"
    png = out_dir / "velocity_z_overlay.png"
    plt.savefig(pdf)
    plt.savefig(png)
    plt.close()


if __name__ == "__main__":
    main()
