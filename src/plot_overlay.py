from pathlib import Path
from typing import Optional, Tuple

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
    truth: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None,
    *,
    t_truth: Optional[np.ndarray] = None,
    pos_truth: Optional[np.ndarray] = None,
    vel_truth: Optional[np.ndarray] = None,
    acc_truth: Optional[np.ndarray] = None,
    suffix: Optional[str] = None,
    include_measurements: bool = True,
) -> None:
    """Save a 3x3 overlay plot comparing measured IMU, measured GNSS and
    fused GNSS+IMU tracks.

    Parameters
    ----------
    t_truth, pos_truth, vel_truth, acc_truth : np.ndarray or None, optional
        Ground-truth time, position, velocity and acceleration samples. When
        provided, a black line labelled ``"Truth"`` is drawn in each subplot.
    suffix : str or None, optional
        Filename suffix appended to ``"{method}_{frame}"`` when saving the
        figure. Defaults to ``"_overlay_truth.pdf"`` if any truth arrays are
        supplied and ``"_overlay.pdf"`` otherwise.
    include_measurements : bool, optional
        Plot measured IMU and GNSS series when ``True`` (default). When ``False``
        only the fused estimate and optional truth data are shown.
    """
    if truth is not None:
        t_truth, pos_truth, vel_truth, acc_truth = truth

    if suffix is None:
        suffix = "_overlay_truth.pdf" if t_truth is not None else "_overlay.pdf"

    axis_labels = {
        "NED": ["\u0394N [m]", "\u0394E [m]", "\u0394D [m]"],
        "ECEF": ["X", "Y", "Z"],
        "Body": ["X", "Y", "Z"],
    }
    cols = axis_labels.get(frame, ["X", "Y", "Z"])

    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)

    datasets = [
        (pos_imu, pos_gnss, pos_fused, pos_truth, "Position [m]"),
        (vel_imu, vel_gnss, vel_fused, vel_truth, "Velocity [m/s]"),
        (acc_imu, acc_gnss, acc_fused, acc_truth, "Acceleration [m/s$^2$]"),
    ]

    for row, (imu, gnss, fused, truth, ylab) in enumerate(datasets):
        for col, axis in enumerate(cols):
            ax = axes[row, col]
            if include_measurements:
                ax.plot(t_gnss, gnss[:, col], "k", label="Measured GNSS")
                ax.plot(t_imu, imu[:, col], "c--", label="Measured IMU")
            if t_truth is not None and truth is not None:
                ax.plot(t_truth, truth[:, col], "m-", label="Truth")
            ax.plot(t_fused, fused[:, col], "g:", label=f"Fused GNSS+IMU ({method})")
            if row == 0:
                ax.set_title(axis)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel("Time [s]")
            ax.legend(loc="best")

    if include_measurements:
        title = f"{method} – {frame} Frame (Fused vs. Measured GNSS)"
    else:
        title = f"{method} – {frame} Frame (Fused vs. Truth)" if t_truth is not None else f"{method} – {frame} Frame (Fused)"
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = Path(out_dir) / f"{method}_{frame}{suffix}"
    fig.savefig(out_path)
    plt.close(fig)
