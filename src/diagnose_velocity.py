"""Diagnostic script for unrealistic velocity divergence.

Usage:
    python diagnose_velocity.py --est-file <file> --truth-file <truth> \
        --imu-file <imu> --gnss-file <gnss> [--frame ECEF] \
        --output-dir results

The script loads fused estimator output, ground truth, IMU and GNSS data,
performs a series of checks and plots to help debug velocity divergence.
If quaternion and time vector lengths differ the data are trimmed to the
shortest length.
All figures are saved to the specified output directory.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import logging
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from validate_with_truth import load_estimate, assemble_frames


logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="Velocity diagnostic")
    parser.add_argument("--est-file", required=True)
    parser.add_argument("--truth-file", required=True)
    parser.add_argument("--imu-file", required=True)
    parser.add_argument("--gnss-file", required=True)
    parser.add_argument("--output-dir", default="results")
    parser.add_argument(
        "--frame",
        choices=["NED", "ECEF", "Body"],
        default="ECEF",
        help="reference frame for comparison",
    )
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    est = load_estimate(args.est_file)
    frames = assemble_frames(est, args.imu_file, args.gnss_file, args.truth_file)
    try:
        t_est, pos_est, vel_est, _ = frames[args.frame]["fused"]
        t_truth, pos_truth, vel_truth, _ = frames[args.frame]["truth"]
    except KeyError:
        raise ValueError(
            f"Truth data not available for frame {args.frame}."
            " Ensure --truth-file is provided."
        )
    quat_est = np.asarray(est.get("quat"))

    if quat_est is not None and len(quat_est) != len(t_est):
        min_len = min(len(quat_est), len(t_est))
        logger.warning(
            "Quaternion length %d does not match time vector %d; trimming to %d",
            len(quat_est),
            len(t_est),
            min_len,
        )
        quat_est = quat_est[:min_len]
        t_est = t_est[:min_len]
        pos_est = pos_est[:min_len]
        vel_est = vel_est[:min_len]
    truth_raw = np.loadtxt(args.truth_file)
    quat_truth = truth_raw[:, 8:12]

    print(f"Estimator position shape: {pos_est.shape}")
    print(f"Estimator velocity shape: {vel_est.shape}")
    print(f"Estimator quaternion shape: {None if quat_est is None else quat_est.shape}")

    if np.any(np.isnan(pos_est)) or np.any(np.isnan(vel_est)):
        print("Warning: NaNs detected in estimator output")

    print(f"Truth position shape: {pos_truth.shape}")
    print(f"Truth velocity shape: {vel_truth.shape}")

    # Reference frame check
    print("First estimator pos:", pos_est[0])
    print("First truth pos:", pos_truth[0])
    diff0 = pos_est[0] - pos_truth[0]
    print(f"Initial pos diff: {diff0}")

    # Interpolate truth to estimator time
    vel_truth_i = np.vstack([np.interp(t_est, t_truth, vel_truth[:, i]) for i in range(3)]).T

    # Velocity diagnostics
    vel_error = vel_est - vel_truth_i
    vel_rmse = float(np.sqrt(np.mean(np.sum(vel_error**2, axis=1))))
    final_vel_err = float(np.linalg.norm(vel_error[-1]))
    print(f"Velocity RMSE: {vel_rmse:.3f} m/s, Final error: {final_vel_err:.3f} m/s")

    # Plots
    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(3, 1, sharex=True)
    for i in range(3):
        axes[i].plot(t_est, vel_est[:, i], label="Fused")
        axes[i].plot(t_est, vel_truth_i[:, i], label="Truth")
        axes[i].set_ylabel(f"{labels[i]} vel [m/s]")
        axes[i].grid(True)
    axes[0].legend()
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "velocity_axes.png")
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t_est, np.linalg.norm(vel_est, axis=1), label="|Fused|")
    ax.plot(t_est, np.linalg.norm(vel_truth_i, axis=1), label="|Truth|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity magnitude [m/s]")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "velocity_norm.png")
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t_est, np.linalg.norm(vel_error, axis=1))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity error [m/s]")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "velocity_error.png")
    plt.close(fig)

    # Attitude diagnostics
    if quat_est is not None:
        eul_est = R.from_quat(quat_est[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
        eul_truth = R.from_quat(quat_truth[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
        print("First 10 estimator Euler angles:")
        print(eul_est[:10])
        fig, axs = plt.subplots(3, 1, sharex=True)
        names = ["Roll", "Pitch", "Yaw"]
        eul_truth_i = np.vstack([
            np.interp(t_est, t_truth, eul_truth[:, i]) for i in range(3)
        ]).T
        for i in range(3):
            axs[i].plot(t_est, eul_est[:, i], label="Fused")
            axs[i].plot(t_est, eul_truth_i[:, i], label="Truth")
            axs[i].set_ylabel(f"{names[i]} [deg]")
            axs[i].grid(True)
        axs[0].legend()
        axs[-1].set_xlabel("Time [s]")
        fig.tight_layout()
        fig.savefig(out_dir / "attitude_euler.png")
        plt.close(fig)

    # IMU diagnostics
    imu = np.loadtxt(args.imu_file)
    if imu.shape[1] >= 10:
        acc = imu[:, 5:8]
        gyro = imu[:, 2:5]
    else:
        acc = imu[:, 2:5]
        gyro = imu[:, 5:8]
    dt = 1.0 / 400.0
    acc = acc / dt
    gyro = gyro / dt
    static_samples = min(4000, len(acc))
    acc_bias = acc[:static_samples].mean(axis=0)
    gyro_bias = gyro[:static_samples].mean(axis=0)
    print(f"Accelerometer bias estimate: {acc_bias}")
    print(f"Gyro bias estimate: {gyro_bias}")

    corrected_acc = acc - acc_bias
    vel_check = np.cumsum(corrected_acc[:static_samples], axis=0) * dt
    print("Velocity after integrating corrected static acceleration:", vel_check[-1])

    # GNSS timeline
    gnss = pd.read_csv(args.gnss_file)
    t_gnss = gnss["Posix_Time"].to_numpy()
    plt.figure()
    plt.plot(t_gnss - t_gnss[0], label="GNSS timestamps")
    plt.plot(t_est - t_est[0], label="Fused timestamps")
    plt.xlabel("Sample")
    plt.ylabel("Time [s]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / "time_alignment.png")
    plt.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    main()
