"""Utilities for standardised 3x3 task plots used in Tasks 4–6.

This module focuses purely on plotting: data preparation should be handled by
upstream code.  The function :func:`plot_task_grid` implements the layout and
styling rules laid out in the repository's plotting guidelines.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from utils.plot_save import save_plot


AxisData = Dict[str, np.ndarray]  # keys: "pos", "vel", "acc"


def _series_info(name: str, t: Optional[np.ndarray], data: Optional[AxisData]) -> bool:
    """Print debug information for a data series.

    Returns ``True`` when the series is missing."""

    if t is None or data is None:
        print(f"[WARN] {name} missing → legend '(missing)'")
        return True
    pos = data.get("pos")
    vel = data.get("vel")
    acc = data.get("acc")
    t_range = (float(t[0]), float(t[-1])) if len(t) else (0.0, 0.0)
    print(
        f"[INFO] {name}: pos {getattr(pos, 'shape', None)}, vel {getattr(vel, 'shape', None)},"
        f" acc {getattr(acc, 'shape', None)}, t len={len(t)} range={t_range}"
    )
    return False


FRAME_AXES = {
    "NED": ["North", "East", "Down"],
    "ECEF": ["X", "Y", "Z"],
    "BODY": ["X", "Y", "Z"],
}

ROW_LABELS = ["Position [m]", "Velocity [m/s]", "Acceleration [m/s²]"]


def plot_task_grid(
    *,
    task: int,
    frame: str,
    dataset: str,
    out_dir: Path | str,
    t_fused: np.ndarray,
    fused: Optional[AxisData],
    t_gnss: Optional[np.ndarray] = None,
    gnss: Optional[AxisData] = None,
    t_imu: Optional[np.ndarray] = None,
    imu: Optional[AxisData] = None,
    t_truth: Optional[np.ndarray] = None,
    truth: Optional[AxisData] = None,
) -> Path:
    """Create a 3×3 plot according to repository rules.

    Parameters are provided as individual time vectors and dictionaries holding
    ``pos``/``vel``/``acc`` arrays with shape ``(N,3)``.
    """

    frame = frame.upper()
    axes_labels = FRAME_AXES.get(frame, ["X", "Y", "Z"])

    print(f"[INFO] Dataset={dataset} task={task} frame={frame}")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] Output directory: {out_dir}")

    missing = []
    if task != 4:
        missing_fused = _series_info("Fused", t_fused, fused)
        if missing_fused:
            missing.append("Fused")
    if task == 4:
        if _series_info("GNSS", t_gnss, gnss):
            missing.append("GNSS")
        if _series_info("IMU only", t_imu, imu):
            missing.append("IMU only")
    if task == 6:
        if _series_info("Truth", t_truth, truth):
            missing.append("Truth")

    if task == 4:
        print("[INFO] using provided time base; interpolation: none")
    else:
        print("[INFO] using fused time base; interpolation: none")

    fig, axes = plt.subplots(3, 3, figsize=(9, 6), sharex=True)
    plt.rcParams.update({"font.size": 12})

    for row, row_label in enumerate(ROW_LABELS):
        for col, axis in enumerate(axes_labels):
            ax = axes[row, col]
            # Plot order: GNSS, IMU, Truth, Fused
            if task == 4:
                if gnss and gnss.get("pos") is not None:
                    gnss_label = "Measured GNSS" if frame == "ECEF" else "Derived GNSS"
                    ax.plot(
                        t_gnss,
                        gnss["pos" if row == 0 else "vel" if row == 1 else "acc"][:, col],
                        linewidth=1.5,
                        label=gnss_label,
                    )
                if imu and imu.get("pos") is not None:
                    imu_label = "Measured IMU" if frame == "BODY" and row == 2 else "Derived IMU"
                    ax.plot(
                        t_imu,
                        imu["pos" if row == 0 else "vel" if row == 1 else "acc"][:, col],
                        linestyle="--",
                        linewidth=1.5,
                        label=imu_label,
                    )
            if task == 6 and truth and truth.get("pos") is not None:
                ax.plot(
                    t_truth,
                    truth["pos" if row == 0 else "vel" if row == 1 else "acc"][:, col],
                    linestyle=":",
                    linewidth=1.5,
                    label="Truth",
                )
            if task != 4 and fused and fused.get("pos") is not None:
                ax.plot(
                    t_fused,
                    fused["pos" if row == 0 else "vel" if row == 1 else "acc"][:, col],
                    linewidth=2.0,
                    label="Fused",
                )
            ax.grid(True)
            ax.set_ylabel(f"{axis} {row_label}")
            if row == 2:
                ax.set_xlabel("Time [s]")

    # build legend
    handles, labels = axes[0, 0].get_legend_handles_labels()
    extra_handles: list[Line2D] = []
    if "GNSS" in missing:
        extra_handles.append(Line2D([], [], color="none", label="GNSS (missing)"))
    if "IMU only" in missing:
        extra_handles.append(Line2D([], [], color="none", label="IMU only (missing)"))
    if "Truth" in missing:
        extra_handles.append(Line2D([], [], color="none", label="Truth (missing)"))
    if "Fused" in missing:
        extra_handles.append(Line2D([], [], color="none", label="Fused (missing)"))
    handles.extend(extra_handles)
    axes[0, 0].legend(handles, labels + [h.get_label() for h in extra_handles], loc="upper right")

    # Titles
    line1 = {
        4: f"Task 4: Comparison ({frame}) — GNSS vs IMU only",
        5: f"Task 5: Fused ({frame})",
        6: f"Task 6: Overlay ({frame}) — Fused vs Truth",
    }[task]

    counts = [f"IMU n={len(t_imu) if t_imu is not None else 0}", f"GNSS n={len(t_gnss) if t_gnss is not None else 0}", f"Truth n={len(t_truth) if t_truth is not None else 0}"]
    line2 = f"{dataset} | {' | '.join(counts)}"
    fig.text(0.5, 0.97, line1, ha="center")
    fig.text(0.5, 0.93, line2, ha="center")
    if missing:
        fig.text(0.5, 0.89, f"⚠ {' ,'.join(missing)} missing", ha="center", color="red")
    fig.tight_layout(rect=[0, 0, 1, 0.86])

    plot_label = {
        4: f"all_{frame.lower()}",
        5: f"all_{frame.lower()}",
        6: f"overlay_{frame.upper()}",
    }[task]
    path = save_plot(fig, out_dir, dataset, f"task{task}", plot_label, dpi=200)
    plt.close(fig)
    return path
