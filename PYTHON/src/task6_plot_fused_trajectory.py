#!/usr/bin/env python3
"""Plot fused IMU/GNSS trajectories against ground truth for Task 6.

Usage:
    python task6_plot_fused_trajectory.py

This script loads the fused estimator output produced in Task 5 together with
matching ground truth and generates overlay plots in both the NED and ECEF
frames.  Legends are added to all subplots and a quaternion comparison plot is
created across TRIAD, SVD and Davenport methods. Plots are written to the
``results/`` directory.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Optional

import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
from velocity_utils import derive_velocity


# ---------------------------------------------------------------------------
# helper functions
# ---------------------------------------------------------------------------


def ned_to_ecef(
    pos_ned: np.ndarray,
    vel_ned: np.ndarray,
    lat_rad: float,
    lon_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert NED position and velocity to the ECEF frame."""
    slat, clat = np.sin(lat_rad), np.cos(lat_rad)
    slon, clon = np.sin(lon_rad), np.cos(lon_rad)
    r_e2n = np.array(
        [
            [-slat * clon, -slat * slon, clat],
            [-slon, clon, 0.0],
            [-clat * clon, -clat * slon, -slat],
        ]
    )
    r_n2e = r_e2n.T
    pos_ecef = pos_ned @ r_n2e
    vel_ecef = vel_ned @ r_n2e
    return pos_ecef, vel_ecef


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------


def plot_task6_fused_trajectory(
    method: str,
    imu_file: str,
    gnss_file: str,
    quat_logs: Optional[Dict[str, np.ndarray]] = None,
) -> Dict[str, np.ndarray]:
    """Plot fused trajectory for *method* and return quaternion logs."""
    fused_path = Path("results") / f"{imu_file}_{gnss_file}_{method}.mat"
    if not fused_path.exists():
        print(f"Error: missing fused data {fused_path}")
        return quat_logs or {}
    truth_path = Path("results") / f"{imu_file}_{gnss_file}_truth.mat"
    if not truth_path.exists():
        print(f"Error: missing truth data {truth_path}")
        return quat_logs or {}

    fused_data = sio.loadmat(fused_path)
    truth_data = sio.loadmat(truth_path)

    try:
        fused_pos_ned = fused_data["fused_pos"]
        fused_vel_ned = fused_data["fused_vel"]
        fused_time = fused_data["time_s"].squeeze()
        ref_lat = float(np.squeeze(fused_data["ref_lat_rad"]))
        ref_lon = float(np.squeeze(fused_data["ref_lon_rad"]))
        quat_log = fused_data.get("quat_log")
    except KeyError as exc:
        print(f"KeyError: {exc} in {fused_path}")
        return quat_logs or {}

    truth_pos_ned = truth_data["pos_ned_m"]
    truth_time = truth_data["time_s"].squeeze()

    if len(truth_time) != len(fused_time) or not np.allclose(truth_time, fused_time):
        truth_pos_ned = np.vstack(
            [np.interp(fused_time, truth_time, truth_pos_ned[:, i]) for i in range(3)]
        ).T
        truth_time = fused_time

    if "vel_ned_ms" in truth_data:
        truth_vel_ned = truth_data["vel_ned_ms"]
        if len(truth_vel_ned) != len(fused_time):
            truth_vel_ned = np.vstack(
                [
                    np.interp(fused_time, truth_time, truth_vel_ned[:, i])
                    for i in range(3)
                ]
            ).T
    else:
        print("Warning: deriving truth velocity from position")
        truth_vel_ned = derive_velocity(truth_time, truth_pos_ned)

    fused_pos_ecef, fused_vel_ecef = ned_to_ecef(
        fused_pos_ned, fused_vel_ned, ref_lat, ref_lon
    )
    truth_pos_ecef, truth_vel_ecef = ned_to_ecef(
        truth_pos_ned, truth_vel_ned, ref_lat, ref_lon
    )

    if quat_logs is not None and quat_log is not None:
        quat_logs[method] = quat_log

    error_pos = fused_pos_ned - truth_pos_ned
    error_vel = fused_vel_ned - truth_vel_ned
    rmse_pos = np.sqrt(np.mean(np.sum(error_pos**2, axis=1)))
    rmse_vel = np.sqrt(np.mean(np.sum(error_vel**2, axis=1)))
    final_err = float(np.linalg.norm(error_pos[-1]))

    for frame_name, (p_f, v_f, p_t, v_t) in {
        "NED": (fused_pos_ned, fused_vel_ned, truth_pos_ned, truth_vel_ned),
        "ECEF": (fused_pos_ecef, fused_vel_ecef, truth_pos_ecef, truth_vel_ecef),
    }.items():
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axis_labels = (
            ["X", "Y", "Z"] if frame_name == "ECEF" else ["North", "East", "Down"]
        )
        for i, lbl in enumerate(axis_labels):
            ax = axes[i]
            ax.plot(fused_time, p_f[:, i], label=f"{method} Fused", color="blue")
            ax.plot(
                fused_time,
                p_t[:, i],
                label="Truth",
                color="red",
                linestyle="--",
            )
            ax.set_ylabel(f"{lbl} [m]")
            ax.grid(True)
            if i == 0:
                ax.legend()
        axes[-1].set_xlabel("Time [s]")
        fig.suptitle(f"Task 6: {method} Position ({frame_name} Frame)")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        out_name = (
            f"{imu_file}_{gnss_file}_{method}_task6_fused_position_{frame_name.lower()}"
        )
        pdf_path = Path("results") / f"{out_name}.pdf"
        from utils.matlab_fig_export import save_matlab_fig
        save_matlab_fig(fig, str(pdf_path.with_suffix('')))
        try:
            from utils import save_plot_mat
            save_plot_mat(fig, str(Path("results") / f"{out_name}.mat"))
        except Exception:
            pass
        plt.close(fig)

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for i, lbl in enumerate(axis_labels):
            ax = axes[i]
            ax.plot(fused_time, v_f[:, i], label=f"{method} Fused", color="blue")
            ax.plot(
                fused_time,
                v_t[:, i],
                label="Truth",
                color="red",
                linestyle="--",
            )
            ax.set_ylabel(f"{lbl} [m/s]")
            ax.grid(True)
            if i == 0:
                ax.legend()
        axes[-1].set_xlabel("Time [s]")
        fig.suptitle(f"Task 6: {method} Velocity ({frame_name} Frame)")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        out_name = (
            f"{imu_file}_{gnss_file}_{method}_task6_fused_velocity_{frame_name.lower()}"
        )
        pdf_path = Path("results") / f"{out_name}.pdf"
        from utils.matlab_fig_export import save_matlab_fig
        save_matlab_fig(fig, str(pdf_path.with_suffix('')))
        try:
            from utils import save_plot_mat
            save_plot_mat(fig, str(Path("results") / f"{out_name}.mat"))
        except Exception:
            pass
        plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, lbl in enumerate(["North", "East", "Down"]):
        ax = axes[i]
        ax.plot(fused_time, error_pos[:, i], label=f"{method} Error", color="green")
        ax.set_ylabel(f"{lbl} Error [m]")
        ax.grid(True)
        if i == 0:
            ax.legend()
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"Task 6: {method} Position Error (NED Frame)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_name = f"{imu_file}_{gnss_file}_{method}_task6_position_error_ned"
    pdf_path = Path("results") / f"{out_name}.pdf"
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(pdf_path.with_suffix('')))
    try:
        from utils import save_plot_mat
        save_plot_mat(fig, str(Path("results") / f"{out_name}.mat"))
    except Exception:
        pass
    plt.close(fig)

    print(
        f"Task 6: {method} final position error {final_err:.3f} m, "
        f"RMSEpos {rmse_pos:.3f} m, RMSEvel {rmse_vel:.3f} m/s"
    )
    return quat_logs or {}


def plot_quaternion_comparison(
    imu_file: str, gnss_file: str, quat_logs: Dict[str, np.ndarray]
) -> None:
    """Plot quaternion components for all methods."""
    if not quat_logs:
        print("No quaternion logs available for comparison")
        return
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    labels = ["qw", "qx", "qy", "qz"]
    colors = ["blue", "red", "green", "purple"]
    for i, (lbl, col) in enumerate(zip(labels, colors)):
        ax = axes[i]
        for method, q in quat_logs.items():
            time_s = sio.loadmat(
                Path("results") / f"{imu_file}_{gnss_file}_{method}.mat"
            )["time_s"].squeeze()
            ax.plot(time_s, q[:, i], label=method)
        ax.set_ylabel(lbl)
        ax.grid(True)
        if i == 0:
            ax.legend()
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Task 6: Quaternion Components")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = (
        Path("results") / f"{imu_file}_{gnss_file}_task6_quaternion_comparison.pdf"
    )
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(out_path.with_suffix('')))
    try:
        from utils import save_plot_mat
        save_plot_mat(fig, str(out_path.with_suffix(".mat")))
    except Exception:
        pass
    plt.close(fig)
    print(f"Saved quaternion comparison to {out_path}")


# ---------------------------------------------------------------------------
# main entry point
# ---------------------------------------------------------------------------


def main() -> None:
    datasets = [("IMU_X002", "GNSS_X002"), ("IMU_X001", "GNSS_X001")]
    methods = ["TRIAD", "SVD", "Davenport"]
    for imu_file, gnss_file in datasets:
        quat_logs: Dict[str, np.ndarray] = {}
        for method in methods:
            print(f"Processing {imu_file}_{gnss_file}_{method}")
            quat_logs = plot_task6_fused_trajectory(
                method, imu_file, gnss_file, quat_logs
            )
        plot_quaternion_comparison(imu_file, gnss_file, quat_logs)


if __name__ == "__main__":
    os.makedirs("results", exist_ok=True)
    main()
