"""Plotting helpers extracted from the original script."""

import matplotlib.pyplot as plt
import numpy as np


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
    plt.savefig(filename)
    plt.close()


def save_euler_angles(t: np.ndarray, euler_angles: np.ndarray) -> None:
    """Plot roll, pitch and yaw over time."""
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.title("Attitude Angles (Roll/Pitch/Yaw) vs. Time")
    plt.savefig("results/attitude_angles_over_time.pdf")
    plt.close()


def save_residual_plots(
    t: np.ndarray,
    pos_filter: np.ndarray,
    pos_gnss: np.ndarray,
    vel_filter: np.ndarray,
    vel_gnss: np.ndarray,
) -> None:
    """Plot aggregated position and velocity residuals."""
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
    plt.savefig("results/position_residuals_vs_time.pdf")
    plt.close()

    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, residual_vel[:, i], label=label)
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity Residual [m/s]")
    plt.title("Velocity Residuals vs. Time")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig("results/velocity_residuals_vs_time.pdf")
    plt.close()


def save_attitude_over_time(t: np.ndarray, euler_angles: np.ndarray) -> None:
    """Plot roll, pitch and yaw over the entire dataset."""
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.title("Attitude Angles (Roll/Pitch/Yaw) Over Time")
    plt.savefig("results/attitude_angles_over_time.pdf")
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
    plt.savefig("results/velocity_profile.pdf")
    plt.close()

