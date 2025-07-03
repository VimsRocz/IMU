from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


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
    truth: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None,
) -> None:
    """Save a 4x1 overlay plot comparing IMU-only, GNSS and fused tracks.

    If *truth* is provided it must be a ``(t, pos, vel, acc)`` tuple and an
    additional green line is plotted for the ground truth data.  In that case
    the output file name gains a ``_truth`` suffix.
    """
    fig, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)

    axes[0].plot(t_imu, _norm(pos_imu), "b--", label="IMU only")
    axes[0].plot(t_gnss, _norm(pos_gnss), "k.", label="GNSS")
    axes[0].plot(t_fused, _norm(pos_fused), "r-", label="Fused")
    if truth is not None:
        t_t, pos_t, _, _ = truth
        axes[0].plot(t_t, _norm(pos_t), "g-", label="Truth")
    axes[0].set_ylabel("Position [m]")
    axes[0].legend()

    axes[1].plot(t_imu, _norm(vel_imu), "b--")
    axes[1].plot(t_gnss, _norm(vel_gnss), "k.")
    axes[1].plot(t_fused, _norm(vel_fused), "r-")
    if truth is not None:
        t_t, _, vel_t, _ = truth
        axes[1].plot(t_t, _norm(vel_t), "g-")
    axes[1].set_ylabel("Velocity [m/s]")

    axes[2].plot(t_imu, _norm(acc_imu), "b--")
    axes[2].plot(t_gnss, _norm(acc_gnss), "k.")
    axes[2].plot(t_fused, _norm(acc_fused), "r-")
    if truth is not None:
        t_t, _, _, acc_t = truth
        axes[2].plot(t_t, _norm(acc_t), "g-")
    axes[2].set_ylabel("Acceleration [m/s$^2$]")

    axes[3].plot(pos_imu[:, 0], pos_imu[:, 1], "b--")
    axes[3].plot(pos_gnss[:, 0], pos_gnss[:, 1], "k.")
    axes[3].plot(pos_fused[:, 0], pos_fused[:, 1], "r-")
    if truth is not None:
        _, pos_t, _, _ = truth
        axes[3].plot(pos_t[:, 0], pos_t[:, 1], "g-")
    axes[3].set_xlabel(f"{frame} X")
    axes[3].set_ylabel(f"{frame} Y")
    axes[3].set_title("Trajectory")
    axes[3].axis("equal")

    fig.suptitle(f"{method} - {frame} frame comparison")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    suffix = "_overlay_truth.pdf" if truth is not None else "_overlay.pdf"
    out_path = Path(out_dir) / f"{method}_{frame}{suffix}"
    fig.savefig(out_path)
    plt.close(fig)
