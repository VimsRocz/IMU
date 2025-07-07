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


def plot_frame(
    frame: str,
    t_gnss: np.ndarray,
    t_imu: np.ndarray,
    gnss_pos_ecef: np.ndarray,
    gnss_vel_ecef: np.ndarray,
    imu_acc_body: np.ndarray,
    fused_pos_ned: np.ndarray,
    fused_vel_ned: np.ndarray,
    fused_acc_ned: np.ndarray,
    C_NED_to_ECEF: np.ndarray,
    C_B_N: np.ndarray,
    out_file: Path,
    *,
    t_truth: Optional[np.ndarray] = None,
    pos_truth: Optional[np.ndarray] = None,
    vel_truth: Optional[np.ndarray] = None,
    acc_truth: Optional[np.ndarray] = None,
    show: bool = True,
) -> None:
    """Plot GNSS, IMU and fused data in a single coordinate frame."""

    gnss_pos = transform_data(
        gnss_pos_ecef, "ECEF", frame, C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF
    )
    gnss_vel = transform_data(
        gnss_vel_ecef, "ECEF", frame, C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF
    )
    imu_acc = transform_data(
        imu_acc_body, "body", frame, C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF
    )
    fused_pos = transform_data(
        fused_pos_ned, "NED", frame, C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF
    )
    fused_vel = transform_data(
        fused_vel_ned, "NED", frame, C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF
    )
    fused_acc = transform_data(
        fused_acc_ned, "NED", frame, C_B_N=C_B_N, C_NED_to_ECEF=C_NED_to_ECEF
    )

    labels = {"NED": ["N", "E", "D"], "ECEF": ["X", "Y", "Z"], "body": ["X", "Y", "Z"]}
    cols = labels.get(frame, ["X", "Y", "Z"])

    fig, axes = plt.subplots(3, 3, figsize=(15, 10))

    for j in range(3):
        axes[0, j].plot(t_gnss, gnss_pos[:, j], "k-", label="GNSS")
        if t_truth is not None and pos_truth is not None:
            axes[0, j].plot(t_truth, pos_truth[:, j], "m-", label="Truth")
        axes[0, j].plot(t_imu, fused_pos[:, j], "b-", label="Fused GNSS + IMU")
        axes[0, j].set_title(f"Position {cols[j]} ({frame})")

        axes[1, j].plot(t_gnss, gnss_vel[:, j], "k-", label="GNSS")
        if t_truth is not None and vel_truth is not None:
            axes[1, j].plot(t_truth, vel_truth[:, j], "m-", label="Truth")
        axes[1, j].plot(t_imu, fused_vel[:, j], "b-", label="Fused GNSS + IMU")
        axes[1, j].set_title(f"Velocity {cols[j]} ({frame})")

        axes[2, j].plot(t_imu, imu_acc[:, j], "g-", label="IMU")
        if t_truth is not None and acc_truth is not None:
            axes[2, j].plot(t_truth, acc_truth[:, j], "m-", label="Truth")
        axes[2, j].plot(t_imu, fused_acc[:, j], "b-", label="Fused GNSS + IMU")
        axes[2, j].set_title(f"Acceleration {cols[j]} ({frame})")

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


def plot_ecef(t_gnss, t_imu, gnss_pos_ecef, gnss_vel_ecef,
              imu_acc_body, fused_pos_ned, fused_vel_ned, fused_acc_ned,
              C_NED_to_ECEF, C_B_N, out_file: Path, *,
              t_truth: Optional[np.ndarray] = None,
              pos_truth: Optional[np.ndarray] = None,
              vel_truth: Optional[np.ndarray] = None,
              acc_truth: Optional[np.ndarray] = None,
              show: bool = True):
    plot_frame(
        "ECEF",
        t_gnss,
        t_imu,
        gnss_pos_ecef,
        gnss_vel_ecef,
        imu_acc_body,
        fused_pos_ned,
        fused_vel_ned,
        fused_acc_ned,
        C_NED_to_ECEF,
        C_B_N,
        out_file,
        t_truth=t_truth,
        pos_truth=pos_truth,
        vel_truth=vel_truth,
        acc_truth=acc_truth,
        show=show,
    )


def plot_ned(t_gnss, t_imu, gnss_pos_ecef, gnss_vel_ecef,
             imu_acc_body, fused_pos_ned, fused_vel_ned, fused_acc_ned,
             C_NED_to_ECEF, C_B_N, out_file: Path, *,
             t_truth: Optional[np.ndarray] = None,
             pos_truth: Optional[np.ndarray] = None,
             vel_truth: Optional[np.ndarray] = None,
             acc_truth: Optional[np.ndarray] = None,
             show: bool = True):
    plot_frame(
        "NED",
        t_gnss,
        t_imu,
        gnss_pos_ecef,
        gnss_vel_ecef,
        imu_acc_body,
        fused_pos_ned,
        fused_vel_ned,
        fused_acc_ned,
        C_NED_to_ECEF,
        C_B_N,
        out_file,
        t_truth=t_truth,
        pos_truth=pos_truth,
        vel_truth=vel_truth,
        acc_truth=acc_truth,
        show=show,
    )


def plot_body(t_gnss, t_imu, gnss_pos_ecef, gnss_vel_ecef,
              imu_acc_body, fused_pos_ned, fused_vel_ned, fused_acc_ned,
              C_NED_to_ECEF, C_B_N, out_file: Path, *,
              t_truth: Optional[np.ndarray] = None,
              pos_truth: Optional[np.ndarray] = None,
              vel_truth: Optional[np.ndarray] = None,
              acc_truth: Optional[np.ndarray] = None,
              show: bool = True):
    plot_frame(
        "body",
        t_gnss,
        t_imu,
        gnss_pos_ecef,
        gnss_vel_ecef,
        imu_acc_body,
        fused_pos_ned,
        fused_vel_ned,
        fused_acc_ned,
        C_NED_to_ECEF,
        C_B_N,
        out_file,
        t_truth=t_truth,
        pos_truth=pos_truth,
        vel_truth=vel_truth,
        acc_truth=acc_truth,
        show=show,
    )
