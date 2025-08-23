"""Plotting helpers extracted from the original script."""

import matplotlib.pyplot as plt
from utils.matlab_fig_export import save_matlab_fig
import numpy as np
from typing import Optional, List, Dict
from naming import plot_path
from pathlib import Path


def save_zupt_variance(
    accel: np.ndarray,
    zupt_mask: np.ndarray,
    dt: float,
    dataset_id: str,
    threshold: float,
    window_size: int = 100,
) -> None:
    """Plot ZUPT-detected intervals and accelerometer variance."""
    t = np.arange(accel.shape[0]) * dt
    accel_norm = np.linalg.norm(accel, axis=1)
    mean_conv = np.ones(window_size) / window_size
    var = np.convolve(accel_norm ** 2, mean_conv, mode="same") - np.convolve(
        accel_norm, mean_conv, mode="same"
    ) ** 2
    plt.figure(figsize=(12, 4))
    plt.plot(t, var, label="Accel Norm Variance", color="tab:blue")
    plt.axhline(threshold, color="gray", linestyle="--", label="ZUPT threshold")
    plt.fill_between(
        t,
        0,
        np.max(var),
        where=zupt_mask,
        color="tab:orange",
        alpha=0.3,
        label="ZUPT Detected",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Variance")
    plt.tight_layout()
    plt.title("ZUPT Detection and Accelerometer Variance")
    filename = f"results/IMU_{dataset_id}_ZUPT_variance.pdf"
    save_matlab_fig(plt.gcf(), str(Path(filename).with_suffix('')))
    plt.close()


def save_euler_angles(
    t: np.ndarray,
    euler_angles: np.ndarray,
    dataset_id: str,
    method: str,
) -> None:
    """Plot roll, pitch and yaw over time.

    Parameters
    ----------
    t : np.ndarray
        Time vector corresponding to ``euler_angles``.
    euler_angles : np.ndarray
        Array of roll, pitch and yaw angles in degrees.
    dataset_id : str
        Identifier of the processed dataset.
    method : str
        Name of the attitude initialisation method.
    """
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.title("Attitude Angles (Roll/Pitch/Yaw) vs. Time")
    filename = f"results/{dataset_id}_{method}_attitude_angles_over_time.pdf"
    save_matlab_fig(plt.gcf(), str(Path(filename).with_suffix('')))
    plt.close()


def save_residual_plots(
    t: np.ndarray,
    pos_filter: np.ndarray,
    pos_gnss: np.ndarray,
    vel_filter: np.ndarray,
    vel_gnss: np.ndarray,
    tag: str,
) -> None:
    """Plot aggregated position and velocity residuals.

    Parameters
    ----------
    t : np.ndarray
        Time vector for the GNSS measurements.
    pos_filter : np.ndarray
        Filtered position in NED frame.
    pos_gnss : np.ndarray
        GNSS derived position in NED frame.
    vel_filter : np.ndarray
        Filtered velocity in NED frame.
    vel_gnss : np.ndarray
        GNSS derived velocity in NED frame.
    tag : str
        Dataset/method tag used as filename prefix.
    """
    residual_pos = pos_filter - pos_gnss
    residual_vel = vel_filter - vel_gnss
    labels = ["North", "East", "Down"]

    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, residual_pos[:, i], label=label)
    plt.xlabel("Time [s]")
    plt.ylabel("Position Residual [m]")
    plt.title("Position Residuals vs. Time")
    plt.legend(loc="best")
    plt.tight_layout()
    filename = plot_path("results", tag, 5, "residuals", "position_residuals")
    save_matlab_fig(plt.gcf(), str(Path(filename).with_suffix('')))
    plt.close()

    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, residual_vel[:, i], label=label)
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity Residual [m/s]")
    plt.title("Velocity Residuals vs. Time")
    plt.legend(loc="best")
    plt.tight_layout()
    filename = plot_path("results", tag, 5, "residuals", "velocity_residuals")
    save_matlab_fig(plt.gcf(), str(Path(filename).with_suffix('')))
    plt.close()


def save_attitude_over_time(
    t: np.ndarray,
    euler_angles: np.ndarray,
    dataset_id: str,
    method: str,
) -> None:
    """Plot roll, pitch and yaw over the entire dataset.

    Parameters
    ----------
    t : np.ndarray
        Time vector corresponding to ``euler_angles``.
    euler_angles : np.ndarray
        Array of roll, pitch and yaw angles in degrees.
    dataset_id : str
        Identifier of the processed dataset.
    method : str
        Name of the attitude initialisation method.
    """
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.title("Attitude Angles (Roll/Pitch/Yaw) Over Time")
    filename = f"results/{dataset_id}_{method}_attitude_angles_over_time.pdf"
    save_matlab_fig(plt.gcf(), str(Path(filename).with_suffix('')))
    plt.close()


def save_velocity_profile(t: np.ndarray, vel_filter: np.ndarray, vel_gnss: np.ndarray) -> None:
    """Plot filter and GNSS velocity over time."""
    labels = ["North", "East", "Down"]
    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, vel_gnss[:, i], linestyle="--", label=f"GNSS {label}")
        plt.plot(t, vel_filter[:, i], label=f"Filter {label}")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Velocity Profile")
    plt.legend(loc="best")
    plt.tight_layout()
    save_matlab_fig(plt.gcf(), "results/velocity_profile")
    plt.close()


def plot_all_methods(
    imu_time: np.ndarray,
    gnss_pos_ned_interp: np.ndarray,
    gnss_vel_ned_interp: np.ndarray,
    gnss_acc_ned_interp: np.ndarray,
    fused_pos: dict,
    fused_vel: dict,
    fused_acc: dict,
    methods: Optional[List[str]] = None,
    colors: Optional[Dict[str, str]] = None,
    savefile: str = "task5_results_all_methods.png",
) -> None:
    """Plot position, velocity and acceleration for all methods in one figure."""

    if methods is None:
        methods = ["TRIAD", "Davenport", "SVD"]
    if colors is None:
        colors = {"TRIAD": "r", "Davenport": "g", "SVD": "b"}

    directions = ["North", "East", "Down"]
    names = ["Position", "Velocity", "Acceleration"]
    ylabels = ["Position (m)", "Velocity (m/s)", "Acceleration (m/s²)"]

    fig, axes = plt.subplots(3, 3, figsize=(18, 10))

    for row, (truth, data_dict) in enumerate(
        zip(
            [gnss_pos_ned_interp, gnss_vel_ned_interp, gnss_acc_ned_interp],
            [fused_pos, fused_vel, fused_acc],
        )
    ):
        for col in range(3):
            ax = axes[row, col]
            ax.plot(imu_time, truth[:, col], "k-", label="GNSS")
            for m in methods:
                if m not in data_dict:
                    continue
                c = colors.get(m, None)
                ax.plot(imu_time, data_dict[m][:, col], c, alpha=0.8, label=m)
            ax.set_title(f"{names[row]} {directions[col]}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabels[row])
            if row == 0 and col == 0:
                ax.legend()

    plt.tight_layout()
    save_matlab_fig(plt.gcf(), str(Path(savefile).with_suffix('')))
    plt.close(fig)
