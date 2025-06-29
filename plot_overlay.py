import os
from pathlib import Path
import numpy as np
try:
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - optional plotting dependency
    plt = None


def _norm(v: np.ndarray) -> np.ndarray:
    return np.linalg.norm(v, axis=1)


def plot_overlay(
    frame: str,
    method: str,
    t_imu: np.ndarray,
    pos_imu: np.ndarray,
    vel_imu: np.ndarray,
    acc_imu: np.ndarray,
    t_gnss: np.ndarray,
    pos_gnss: np.ndarray,
    vel_gnss: np.ndarray,
    acc_gnss: np.ndarray,
    t_fused: np.ndarray,
    pos_fused: np.ndarray,
    vel_fused: np.ndarray,
    acc_fused: np.ndarray,
    out_dir: str,
) -> None:
    """Save a 4x1 overlay plot comparing IMU-only, GNSS and fused tracks."""
    fig, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)

    axes[0].plot(t_imu, _norm(pos_imu), "b--", label="IMU only")
    axes[0].plot(t_gnss, _norm(pos_gnss), "k.", label="GNSS")
    axes[0].plot(t_fused, _norm(pos_fused), "r-", label="Fused")
    axes[0].set_ylabel("Position [m]")
    axes[0].legend()

    axes[1].plot(t_imu, _norm(vel_imu), "b--")
    axes[1].plot(t_gnss, _norm(vel_gnss), "k.")
    axes[1].plot(t_fused, _norm(vel_fused), "r-")
    axes[1].set_ylabel("Velocity [m/s]")

    axes[2].plot(t_imu, _norm(acc_imu), "b--")
    axes[2].plot(t_gnss, _norm(acc_gnss), "k.")
    axes[2].plot(t_fused, _norm(acc_fused), "r-")
    axes[2].set_ylabel("Acceleration [m/s$^2$]")

    axes[3].plot(pos_imu[:, 0], pos_imu[:, 1], "b--")
    axes[3].plot(pos_gnss[:, 0], pos_gnss[:, 1], "k.")
    axes[3].plot(pos_fused[:, 0], pos_fused[:, 1], "r-")
    axes[3].set_xlabel(f"{frame} X")
    axes[3].set_ylabel(f"{frame} Y")
    axes[3].set_title("Trajectory")
    axes[3].axis("equal")

    fig.suptitle(f"{method} - {frame} frame comparison")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out_path = Path(out_dir) / f"{method}_{frame}_overlay.pdf"
    fig.savefig(out_path)
    plt.close(fig)
