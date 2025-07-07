import logging
from pathlib import Path
from typing import Optional
import numpy as np
import matplotlib.pyplot as plt


def transform_data(data, from_frame: str, to_frame: str,
                   C_B_N=None, C_NED_to_ECEF=None):
    """Convert position, velocity or acceleration between frames."""
    if from_frame == to_frame:
        return data
    if from_frame == "ECEF" and to_frame == "NED":
        C_ECEF_to_NED = C_NED_to_ECEF.T
        return np.array([C_ECEF_to_NED @ d for d in data])
    if from_frame == "NED" and to_frame == "ECEF":
        return np.array([C_NED_to_ECEF @ d for d in data])
    if from_frame == "body" and to_frame == "NED":
        return np.array([C_B_N @ d for d in data])
    if from_frame == "NED" and to_frame == "body":
        return np.array([(C_B_N.T) @ d for d in data])
    if from_frame == "body" and to_frame == "ECEF":
        C = C_NED_to_ECEF @ C_B_N
        return np.array([C @ d for d in data])
    if from_frame == "ECEF" and to_frame == "body":
        C = (C_B_N.T) @ (C_NED_to_ECEF.T)
        return np.array([C @ d for d in data])
    raise ValueError(f"Unsupported transformation from {from_frame} to {to_frame}")



def _plot_components(ax_arr, t_data, data_arr, label: str, color: str = None):
    for i in range(3):
        ax_arr[i].plot(t_data, data_arr[:, i], label=label, color=color)


def plot_ecef(t_gnss, t_imu, gnss_pos_ecef, gnss_vel_ecef,
              imu_acc_body, fused_pos_ned, fused_vel_ned, fused_acc_ned,
              imu_pos_ned, imu_vel_ned,
              C_NED_to_ECEF, C_B_N, method: str, out_file: Path, *,
              t_truth: Optional[np.ndarray] = None,
              pos_truth: Optional[np.ndarray] = None,
              vel_truth: Optional[np.ndarray] = None,
              acc_truth: Optional[np.ndarray] = None,
              show: bool = True):
    pos_fused = transform_data(fused_pos_ned, "NED", "ECEF", C_NED_to_ECEF=C_NED_to_ECEF)
    vel_fused = transform_data(fused_vel_ned, "NED", "ECEF", C_NED_to_ECEF=C_NED_to_ECEF)
    acc_fused = transform_data(fused_acc_ned, "NED", "ECEF", C_NED_to_ECEF=C_NED_to_ECEF)
    imu_acc_ecef = transform_data(imu_acc_body, "body", "ECEF", C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF)
    imu_pos_ecef = transform_data(imu_pos_ned, "NED", "ECEF", C_NED_to_ECEF=C_NED_to_ECEF)
    imu_vel_ecef = transform_data(imu_vel_ned, "NED", "ECEF", C_NED_to_ECEF=C_NED_to_ECEF)

    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    labels = ["X", "Y", "Z"]

    for j in range(3):
        axes[0, j].plot(t_gnss, gnss_pos_ecef[:, j], 'k-', label="GNSS (Measured)")
        if t_truth is not None and pos_truth is not None:
            axes[0, j].plot(t_truth, pos_truth[:, j], 'm-', label="Truth")
        axes[0, j].plot(t_imu, imu_pos_ecef[:, j], 'g--', label="IMU (Derived)")
        axes[0, j].plot(t_imu, pos_fused[:, j], 'b-', label=f"Fused {method}")
        axes[0, j].set_title(f"Position {labels[j]} (ECEF)")

        axes[1, j].plot(t_gnss, gnss_vel_ecef[:, j], 'k-', label="GNSS (Measured)")
        if t_truth is not None and vel_truth is not None:
            axes[1, j].plot(t_truth, vel_truth[:, j], 'm-', label="Truth")
        axes[1, j].plot(t_imu, imu_vel_ecef[:, j], 'g--', label="IMU (Derived)")
        axes[1, j].plot(t_imu, vel_fused[:, j], 'b-', label=f"Fused {method}")
        axes[1, j].set_title(f"Velocity {labels[j]} (ECEF)")

        axes[2, j].plot(t_imu, imu_acc_ecef[:, j], 'g-', label="IMU (Derived)")
        if t_truth is not None and acc_truth is not None:
            axes[2, j].plot(t_truth, acc_truth[:, j], 'm-', label="Truth")
        axes[2, j].plot(t_imu, acc_fused[:, j], 'b-', label=f"Fused {method}")
        axes[2, j].set_title(f"Acceleration {labels[j]} (ECEF)")

    for ax_row in axes:
        for ax in ax_row:
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Value")
            ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(out_file)
    if show:
        logging.info("Saved %s", out_file)
    plt.close(fig)


def plot_ned(t_gnss, t_imu, gnss_pos_ecef, gnss_vel_ecef,
             imu_acc_body, fused_pos_ned, fused_vel_ned, fused_acc_ned,
             imu_pos_ned, imu_vel_ned,
             C_NED_to_ECEF, C_B_N, method: str, out_file: Path, *,
             t_truth: Optional[np.ndarray] = None,
             pos_truth: Optional[np.ndarray] = None,
             vel_truth: Optional[np.ndarray] = None,
             acc_truth: Optional[np.ndarray] = None,
             show: bool = True):
    gnss_pos_ned = transform_data(gnss_pos_ecef, "ECEF", "NED", C_NED_to_ECEF=C_NED_to_ECEF)
    gnss_vel_ned = transform_data(gnss_vel_ecef, "ECEF", "NED", C_NED_to_ECEF=C_NED_to_ECEF)
    imu_acc_ned = transform_data(imu_acc_body, "body", "NED", C_B_N=C_B_N)
    imu_pos_ned = np.asarray(imu_pos_ned)
    imu_vel_ned = np.asarray(imu_vel_ned)

    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    labels = ["N", "E", "D"]

    for j in range(3):
        axes[0, j].plot(t_gnss, gnss_pos_ned[:, j], 'k-', label="GNSS (Derived)")
        if t_truth is not None and pos_truth is not None:
            axes[0, j].plot(t_truth, pos_truth[:, j], 'm-', label="Truth")
        axes[0, j].plot(t_imu, imu_pos_ned[:, j], 'g--', label="IMU (Derived)")
        axes[0, j].plot(t_imu, fused_pos_ned[:, j], 'b-', label=f"Fused {method}")
        axes[0, j].set_title(f"Position {labels[j]} (NED)")

        axes[1, j].plot(t_gnss, gnss_vel_ned[:, j], 'k-', label="GNSS (Derived)")
        if t_truth is not None and vel_truth is not None:
            axes[1, j].plot(t_truth, vel_truth[:, j], 'm-', label="Truth")
        axes[1, j].plot(t_imu, imu_vel_ned[:, j], 'g--', label="IMU (Derived)")
        axes[1, j].plot(t_imu, fused_vel_ned[:, j], 'b-', label=f"Fused {method}")
        axes[1, j].set_title(f"Velocity {labels[j]} (NED)")

        axes[2, j].plot(t_imu, imu_acc_ned[:, j], 'g-', label="IMU (Derived)")
        if t_truth is not None and acc_truth is not None:
            axes[2, j].plot(t_truth, acc_truth[:, j], 'm-', label="Truth")
        axes[2, j].plot(t_imu, fused_acc_ned[:, j], 'b-', label=f"Fused {method}")
        axes[2, j].set_title(f"Acceleration {labels[j]} (NED)")

    for ax_row in axes:
        for ax in ax_row:
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Value")
            ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(out_file)
    if show:
        logging.info("Saved %s", out_file)
    plt.close(fig)


def plot_body(t_gnss, t_imu, gnss_pos_ecef, gnss_vel_ecef,
              imu_acc_body, fused_pos_ned, fused_vel_ned, fused_acc_ned,
              imu_pos_ned, imu_vel_ned,
              C_NED_to_ECEF, C_B_N, method: str, out_file: Path, *,
              t_truth: Optional[np.ndarray] = None,
              pos_truth: Optional[np.ndarray] = None,
              vel_truth: Optional[np.ndarray] = None,
              acc_truth: Optional[np.ndarray] = None,
              show: bool = True):
    gnss_pos_body = transform_data(gnss_pos_ecef, "ECEF", "body",
                                   C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF)
    gnss_vel_body = transform_data(gnss_vel_ecef, "ECEF", "body",
                                   C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF)
    fused_pos_body = transform_data(fused_pos_ned, "NED", "body", C_B_N=C_B_N)
    fused_vel_body = transform_data(fused_vel_ned, "NED", "body", C_B_N=C_B_N)
    fused_acc_body = transform_data(fused_acc_ned, "NED", "body", C_B_N=C_B_N)
    imu_pos_body = transform_data(imu_pos_ned, "NED", "body", C_B_N=C_B_N)
    imu_vel_body = transform_data(imu_vel_ned, "NED", "body", C_B_N=C_B_N)

    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    labels = ["X", "Y", "Z"]

    for j in range(3):
        axes[0, j].plot(t_gnss, gnss_pos_body[:, j], 'k-', label="GNSS (Derived)")
        if t_truth is not None and pos_truth is not None:
            axes[0, j].plot(t_truth, pos_truth[:, j], 'm-', label="Truth")
        axes[0, j].plot(t_imu, imu_pos_body[:, j], 'g--', label="IMU (Derived)")
        axes[0, j].plot(t_imu, fused_pos_body[:, j], 'b-', label=f"Fused {method}")
        axes[0, j].set_title(f"Position {labels[j]} (Body)")

        axes[1, j].plot(t_gnss, gnss_vel_body[:, j], 'k-', label="GNSS (Derived)")
        if t_truth is not None and vel_truth is not None:
            axes[1, j].plot(t_truth, vel_truth[:, j], 'm-', label="Truth")
        axes[1, j].plot(t_imu, imu_vel_body[:, j], 'g--', label="IMU (Derived)")
        axes[1, j].plot(t_imu, fused_vel_body[:, j], 'b-', label=f"Fused {method}")
        axes[1, j].set_title(f"Velocity {labels[j]} (Body)")

        axes[2, j].plot(t_imu, imu_acc_body[:, j], 'g-', label="IMU (Measured)")
        if t_truth is not None and acc_truth is not None:
            axes[2, j].plot(t_truth, acc_truth[:, j], 'm-', label="Truth")
        axes[2, j].plot(t_imu, fused_acc_body[:, j], 'b-', label=f"Fused {method}")
        axes[2, j].set_title(f"Acceleration {labels[j]} (Body)")

    for ax_row in axes:
        for ax in ax_row:
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Value")
            ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(out_file)
    if show:
        logging.info("Saved %s", out_file)
    plt.close(fig)
