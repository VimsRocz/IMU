import numpy as np
from pathlib import Path
from typing import Optional
import logging

try:
    import matplotlib.pyplot as plt
except Exception as e:  # pragma: no cover - matplotlib optional
    logging.warning("matplotlib not available, plot functions disabled: %s", e)
    plt = None


def plot_frame(
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
    truth: Optional[tuple] = None,
) -> None:
    """Plot comparison for one frame.

    Parameters
    ----------
    frame : str
        Name of the frame (``NED``, ``ECEF`` or ``Body``).
    method : str
        Name of the initialisation method.
    out_dir : str
        Directory where the PNG will be written.
    truth : optional tuple
        ``(t, pos, vel, acc)`` arrays for the ground truth.
    """
    if plt is None:
        return

    labels = {
        "NED": ["N", "E", "D"],
        "ECEF": ["X", "Y", "Z"],
        "Body": ["X", "Y", "Z"],
    }.get(frame.upper(), ["X", "Y", "Z"])

    fig, axes = plt.subplots(3, 3, figsize=(12, 8), sharex=False)

    def _plot_row(ax, t_truth, data_truth, t1, d1, t2, d2, t3, d3, ylabel, title):
        for j, lab in enumerate(labels):
            ax[j].plot(t_gnss, d1[:, j], "k-", label="GNSS")
            ax[j].plot(t_imu, d2[:, j], "b--", label="IMU")
            ax[j].plot(t_fused, d3[:, j], "r-", label="Fused")
            if t_truth is not None:
                ax[j].plot(t_truth, data_truth[:, j], "g-", label="Truth")
            ax[j].set_title(f"{title} {lab}")
            ax[j].set_xlabel("Time [s]")
            ax[j].set_ylabel(ylabel)
            ax[j].legend()

    if truth is not None:
        t_truth, pos_truth, vel_truth, acc_truth = truth
    else:
        t_truth = pos_truth = vel_truth = acc_truth = None

    _plot_row(axes[0], t_truth, pos_truth, t_gnss, pos_gnss, t_imu, pos_imu, t_fused, pos_fused, "[m]", "Position")
    _plot_row(axes[1], t_truth, vel_truth, t_gnss, vel_gnss, t_imu, vel_imu, t_fused, vel_fused, "[m/s]", "Velocity")
    _plot_row(axes[2], t_truth, acc_truth, t_gnss, acc_gnss, t_imu, acc_imu, t_fused, acc_fused, "[m/s$^2$]", "Acceleration")

    fig.suptitle(f"{method} comparison in {frame} frame")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"Task5_compare_{frame.upper()}.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
