#!/usr/bin/env python3
"""Overlay plot of fused GNSS+IMU estimate versus ground truth for Task 6.

Usage:
    python task6_overlay_plot.py --est-file <fused_estimate.npz> \
        --truth-file <STATE_X001.txt> --method TRIAD --frame ECEF \
        --dataset IMU_X002_GNSS_X002

This script synchronizes the time bases of the estimator output and ground
truth, then generates a single figure with position, velocity and acceleration
components overlaid. Only the fused estimate and truth are shown. The figure is
saved to ``results/{DATASET}_{METHOD}_Task6_{FRAME}_Overlay.pdf`` and ``.png``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


# ---------------------------------------------------------------------------
# helper functions
# ---------------------------------------------------------------------------

def load_truth(path: Path, frame: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load ground truth file ``STATE_X*.txt``.

    Parameters
    ----------
    path : Path
        Path to the truth file (txt or npz).
    frame : {"ECEF", "NED"}
        Frame for the output arrays.

    Returns
    -------
    time : ndarray of shape (N,)
    pos : ndarray of shape (N, 3)
    vel : ndarray of shape (N, 3)
    acc : ndarray of shape (N, 3)
    """
    if path.suffix == ".npz":
        data = np.load(path)
        if frame == "ECEF":
            pos = data["pos_ecef_m"]
            vel = data["vel_ecef_ms"]
        else:
            pos = data["pos_ned_m"]
            vel = data["vel_ned_ms"]
        time = data["time_s"]
    else:
        raw = np.loadtxt(path)
        time = raw[:, 1]
        if frame == "ECEF":
            pos = raw[:, 2:5]
            vel = raw[:, 5:8]
        else:
            pos = raw[:, 9:12]
            vel = raw[:, 12:15]
    acc = np.gradient(np.gradient(pos, axis=0), axis=0) / np.diff(time).mean() ** 2
    return time, pos, vel, acc


def load_estimate(path: Path, frame: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load fused estimate ``.npz`` file."""
    data = np.load(path)
    time = data["time_s"]
    if frame == "ECEF":
        pos = data["pos_ecef_m"]
        vel = data["vel_ecef_ms"]
    else:
        pos = data["pos_ned_m"]
        vel = data["vel_ned_ms"]
    if "acc_ecef_ms2" in data:
        acc = data["acc_ecef_ms2"] if frame == "ECEF" else data["acc_ned_ms2"]
    else:
        acc = np.gradient(np.gradient(pos, axis=0), axis=0) / np.diff(time).mean() ** 2
    return time, pos, vel, acc


def interpolate_truth(t_est: np.ndarray, t_truth: np.ndarray, arr: np.ndarray) -> np.ndarray:
    """Interpolate ``arr`` to estimator time vector."""
    f = interp1d(t_truth, arr, axis=0, bounds_error=False, fill_value="extrapolate")
    return f(t_est)


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------

def plot_overlay(t_est: np.ndarray, pos_est: np.ndarray, vel_est: np.ndarray, acc_est: np.ndarray,
                 t_truth: np.ndarray, pos_truth: np.ndarray, vel_truth: np.ndarray, acc_truth: np.ndarray,
                 frame: str, method: str, dataset: str, out_dir: Path) -> Path:
    """Create the overlay plot and save it to ``out_dir``."""
    labels = ["X", "Y", "Z"] if frame == "ECEF" else ["N", "E", "D"]
    colors = ["#377eb8", "#e41a1c", "#4daf4a"]  # colorblind friendly

    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    for ax, est, truth, ylab in zip(
        axes,
        (pos_est, vel_est, acc_est),
        (pos_truth, vel_truth, acc_truth),
        ("Position [m]", "Velocity [m/s]", "Acceleration [m/s$^2$]"),
    ):
        for i in range(3):
            ax.plot(t_est, est[:, i], color=colors[i], label=f"Fused {labels[i]}")
            ax.plot(t_est, truth[:, i], "--", color=colors[i], label=f"Truth {labels[i]}")
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time [s]")
    axes[0].set_title(f"{dataset} Task 6 Overlay — {method} ({frame} frame)")
    axes[0].legend(loc="upper right", ncol=3, fontsize=9, frameon=True)
    for ax in axes[1:]:
        ax.legend().set_visible(False)

    fig.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / f"{dataset}_{method}_Task6_{frame}_Overlay.pdf"
    png_path = out_dir / f"{dataset}_{method}_Task6_{frame}_Overlay.png"
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    plt.close(fig)
    print(f"Saved overlay figure to {pdf_path}")
    return pdf_path


# ---------------------------------------------------------------------------
# main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Generate Task 6 overlay plot of fused estimate vs truth")
    ap.add_argument("--est-file", required=True, help="fused estimator .npz file")
    ap.add_argument("--truth-file", required=True, help="ground truth STATE_X file")
    ap.add_argument("--method", required=True, help="estimation method name")
    ap.add_argument("--frame", choices=["ECEF", "NED"], default="ECEF", help="reference frame")
    ap.add_argument("--dataset", required=True, help="dataset identifier for filename")
    ap.add_argument("--output-dir", default="results", help="directory for saved figure")
    args = ap.parse_args()

    est_path = Path(args.est_file)
    truth_path = Path(args.truth_file)
    out_dir = Path(args.output_dir)

    t_est, pos_est, vel_est, acc_est = load_estimate(est_path, args.frame)
    t_truth, pos_truth, vel_truth, acc_truth = load_truth(truth_path, args.frame)

    pos_truth_i = interpolate_truth(t_est, t_truth, pos_truth)
    vel_truth_i = interpolate_truth(t_est, t_truth, vel_truth)
    acc_truth_i = interpolate_truth(t_est, t_truth, acc_truth)

    plot_overlay(
        t_est,
        pos_est,
        vel_est,
        acc_est,
        t_est,
        pos_truth_i,
        vel_truth_i,
        acc_truth_i,
        args.frame,
        args.method,
        args.dataset,
        out_dir,
    )


if __name__ == "__main__":
    main()
