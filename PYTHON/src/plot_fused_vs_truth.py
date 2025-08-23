"""Simple Task 6 plotting utility for fused vs truth trajectories."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np
import matplotlib.pyplot as plt


def plot_fused_vs_truth(
    time: Iterable[float],
    truth: np.ndarray,
    fused: np.ndarray,
    output_path: str | Path,
    *,
    frame_label: str | None = None,
    unit: str = "m",
) -> None:
    """Plot fused vs truth series for Task 6.

    Parameters
    ----------
    time : array-like of shape (N,)
        Time vector in seconds.
    truth : array-like of shape (N, 3)
        Reference position/velocity/acceleration.
    fused : array-like of shape (N, 3)
        Fused GNSS+IMU estimate in the same frame as ``truth``.
    output_path : str or Path
        Where to write the generated figure (PDF or PNG).
    frame_label : str, optional
        Name of the reference frame for the title, e.g. ``"ECEF"``.
    unit : str, optional
        Unit string appended to the axis labels (default ``"m"``).
    """

    time = np.asarray(time).squeeze()
    truth = np.asarray(truth)
    fused = np.asarray(fused)

    if truth.shape != fused.shape:
        n = min(truth.shape[0], fused.shape[0])
        print(f"Warning: data length mismatch, trimming to {n} samples")
        truth = truth[:n]
        fused = fused[:n]
        time = time[:n]
    elif len(time) != truth.shape[0]:
        n = min(len(time), truth.shape[0])
        print(f"Warning: time length mismatch, trimming to {n} samples")
        time = time[:n]
        truth = truth[:n]
        fused = fused[:n]

    labels = {
        "NED": ["North", "East", "Down"],
        "ECEF": ["X", "Y", "Z"],
        "Body": ["X", "Y", "Z"],
    }
    axes_labels = labels.get(frame_label or "", ["X", "Y", "Z"])

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for i, axis in enumerate(axes_labels):
        axes[i].plot(time, truth[:, i], "k--", linewidth=2, label="Truth")
        axes[i].plot(
            time,
            fused[:, i],
            "r-",
            linewidth=2,
            label="Fused GNSS+IMU (TRIAD)",
        )
        axes[i].set_ylabel(f"{axis} [{unit}]")
        axes[i].grid(True)
    axes[-1].set_xlabel("Time [s]")

    handles, labels_ = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc="upper center", ncol=2, frameon=False)

    title = "Task 6 Comparison"
    if frame_label:
        title += f": {frame_label} Frame"
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(output_path.with_suffix('')))
    plt.close(fig)
