from pathlib import Path
from typing import Optional, Tuple

from matplotlib.lines import Line2D

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
    filename: Optional[str] = None,
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
    filename : str or None, optional
        Full filename (relative to ``out_dir``) for the saved figure. When
        provided, overrides the ``method``/``frame`` naming scheme and the
        ``suffix`` parameter.
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

    legend_handles = []
    legend_labels = []

    def add_handle(handle, label):
        if label not in legend_labels:
            legend_handles.append(handle)
            legend_labels.append(label)

    has_acc_truth = acc_truth is not None and len(acc_truth) > 0

    for row, (imu, gnss, fused, truth, ylab) in enumerate(datasets):
        for col, axis in enumerate(cols):
            ax = axes[row, col]
            if include_measurements:
                (h,) = ax.plot(t_gnss, gnss[:, col], "k", label="Measured GNSS")
                add_handle(h, "Measured GNSS")
                (h,) = ax.plot(t_imu, imu[:, col], "c--", label="Measured IMU")
                add_handle(h, "Measured IMU")
            if (
                t_truth is not None
                and truth is not None
                and not (row == 2 and not has_acc_truth)
            ):
                (h,) = ax.plot(t_truth, truth[:, col], "m-", label="Truth")
                add_handle(h, "Truth")
            (h,) = ax.plot(
                t_fused, fused[:, col], "g:", label=f"Fused GNSS+IMU ({method})"
            )
            add_handle(h, f"Fused GNSS+IMU ({method})")
            if row == 0:
                ax.set_title(axis)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel("Time [s]")

    if t_truth is not None:
        title = f"Task 6 – {method} – {frame} Frame (Fused vs. Truth)"
    elif include_measurements:
        title = f"Task 6 – {method} – {frame} Frame (Fused vs. Measured GNSS)"
    else:
        title = f"Task 6 – {method} – {frame} Frame (Fused)"

    if not has_acc_truth and t_truth is not None:
        add_handle(
            Line2D([], [], color="none", label="No acceleration for Truth"),
            "No acceleration for Truth",
        )

    fig.legend(
        legend_handles,
        legend_labels,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.05),
        ncol=len(legend_labels),
        frameon=False,
    )
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.9])
    if filename is not None:
        out_path = Path(out_dir) / filename
    else:
        out_path = Path(out_dir) / f"{method}_{frame}{suffix}"
    fig.savefig(out_path)
    print(f"Saved overlay figure {out_path}")
    plt.close(fig)
