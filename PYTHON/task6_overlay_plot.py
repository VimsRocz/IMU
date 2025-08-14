#!/usr/bin/env python3
"""Overlay plot of fused GNSS+IMU estimate versus ground truth for Task 6.

Usage:
    python task6_overlay_plot.py --est-file <fused_estimate.npz> \
        --truth-file <STATE_X001.txt> --method TRIAD --frame ECEF \
        --dataset IMU_X002_GNSS_X002 [--debug]

This script synchronizes the time bases of the estimator output and ground
truth, then generates a single figure with position, velocity and acceleration
components overlaid. Only the fused estimate and truth are shown. Figures are
written under ``results/<run_id>/`` using the filename pattern
``<run_id>_task6_overlay_state_<frame>.pdf`` where ``run_id`` combines the
dataset and method, e.g. ``IMU_X003_GNSS_X002_TRIAD``.
With ``--debug`` the script prints diagnostic information about the input
datasets before plotting.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import warnings


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


def load_estimate(
    path: Path, frame: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    """Load fused estimate ``.npz`` file.``None`` if mandatory keys missing."""
    data = np.load(path)
    required = ["time_s"]
    if frame == "ECEF":
        required += ["pos_ecef_m", "vel_ecef_ms"]
    else:
        required += ["pos_ned_m", "vel_ned_ms"]
    missing = [k for k in required if k not in data]
    if missing:
        warnings.warn(
            f"{', '.join(missing)} missing in {path.name}; skipping overlay",
            RuntimeWarning,
        )
        return None

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


def print_debug_info(
    t_est: np.ndarray, vel_est: np.ndarray, t_truth: np.ndarray, vel_truth: np.ndarray
) -> None:
    """Print dataset statistics to aid alignment debugging."""
    print(f"Length of fused velocity: {vel_est.shape}")
    print(f"Length of truth velocity: {vel_truth.shape}")
    print(f"Fused time range: {t_est[0]:.3f} to {t_est[-1]:.3f}")
    print(f"Truth time range: {t_truth[0]:.3f} to {t_truth[-1]:.3f}")
    print("First 5 fused velocity X:", vel_est[:5, 0])
    print("First 5 truth velocity X:", vel_truth[:5, 0])
    print("Last 5 fused velocity X:", vel_est[-5:, 0])
    print("Last 5 truth velocity X:", vel_truth[-5:, 0])
    dv_truth = np.diff(vel_truth[:, 0])
    dv_est = np.diff(vel_est[:, 0])
    print("Max velocity jump in truth (X):", np.max(np.abs(dv_truth)))
    print("Max velocity jump in fused (X):", np.max(np.abs(dv_est)))


# ---------------------------------------------------------------------------
# length handling
# ---------------------------------------------------------------------------


def ensure_equal_length(
    t_est: np.ndarray,
    pos_est: np.ndarray,
    vel_est: np.ndarray,
    acc_est: np.ndarray,
    pos_truth_i: np.ndarray,
    vel_truth_i: np.ndarray,
    acc_truth_i: np.ndarray,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """Truncate arrays to the common minimum length."""
    if len(pos_est) != len(pos_truth_i):
        n = min(len(pos_est), len(pos_truth_i))
        warnings.warn(
            f"Length mismatch after interpolation (est {len(pos_est)}, truth {len(pos_truth_i)}); truncating to {n}",
            RuntimeWarning,
        )
        t_est = t_est[:n]
        pos_est = pos_est[:n]
        vel_est = vel_est[:n]
        acc_est = acc_est[:n]
        pos_truth_i = pos_truth_i[:n]
        vel_truth_i = vel_truth_i[:n]
        acc_truth_i = acc_truth_i[:n]
    assert len(pos_est) == len(pos_truth_i)
    return (
        t_est,
        pos_est,
        vel_est,
        acc_est,
        pos_truth_i,
        vel_truth_i,
        acc_truth_i,
    )


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------


def plot_overlay(
    t_est: np.ndarray,
    pos_est: np.ndarray,
    vel_est: np.ndarray,
    acc_est: np.ndarray,
    t_truth: np.ndarray,
    pos_truth: np.ndarray,
    vel_truth: np.ndarray,
    acc_truth: np.ndarray,
    frame: str,
    method: str,
    dataset: str,
    out_dir: Path,
) -> Path:
    """Create the overlay plot and save it to ``out_dir``.

    The figure uses a consistent legend scheme: the fused estimate is shown
    in a red solid line with ``"x"`` markers, and the ground truth is shown
    in a black dotted line with ``"*"`` markers.
    """
    labels = ["X", "Y", "Z"] if frame == "ECEF" else ["N", "E", "D"]

    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)

    datasets = [
        (pos_est, pos_truth, "Position [m]"),
        (vel_est, vel_truth, "Velocity [m/s]"),
    ]

    for row, (est, truth, ylab) in enumerate(datasets):
        for col in range(3):
            ax = axes[row, col]
            # plot fused estimate in red with 'x' marker
            label_fused = "Fused" if (row == 0 and col == 0) else "_nolegend_"
            ax.plot(
                t_est,
                est[:, col],
                "r-",
                linewidth=2,
                marker="x",
                markersize=3,
                label=label_fused,
            )
            # plot truth in black dotted with '*' marker
            label_truth = "Truth" if (row == 0 and col == 0) else "_nolegend_"
            ax.plot(
                t_est,
                truth[:, col],
                "k:",
                marker="*",
                markersize=3,
                label=label_truth,
            )
            ax.set_ylabel(ylab if col == 0 else "")
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title(labels[col])
            ax.set_xlabel("Time [s]")

    axes[0, 0].legend(loc="upper right", fontsize=9, frameon=True)

    fig.suptitle(f"{dataset} Task 6 Overlay — {method} ({frame} frame)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    run_id = f"{dataset}_{method}"
    pdf_path = out_dir / f"{run_id}_task6_overlay_state_{frame}.pdf"
    png_path = out_dir / f"{run_id}_task6_overlay_state_{frame}.png"
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    print(f"[SAVE] {pdf_path}")
    print(f"[SAVE] {png_path}")
    plt.close(fig)
    return pdf_path


def plot_rmse(
    t: np.ndarray,
    pos_est: np.ndarray,
    vel_est: np.ndarray,
    acc_est: np.ndarray,
    pos_truth: np.ndarray,
    vel_truth: np.ndarray,
    acc_truth: np.ndarray,
    frame: str,
    method: str,
    dataset: str,
    out_dir: Path,
) -> Path:
    """Plot total error magnitude and annotate RMSE values."""
    pos_err = np.linalg.norm(pos_est - pos_truth, axis=1)
    vel_err = np.linalg.norm(vel_est - vel_truth, axis=1)
    acc_err = np.linalg.norm(acc_est - acc_truth, axis=1)

    rmse_pos = float(np.sqrt(np.mean(pos_err**2)))
    rmse_vel = float(np.sqrt(np.mean(vel_err**2)))
    rmse_acc = float(np.sqrt(np.mean(acc_err**2)))

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, pos_err, label=f"Pos RMSE {rmse_pos:.3f} m")
    ax.plot(t, vel_err, label=f"Vel RMSE {rmse_vel:.3f} m/s")
    ax.plot(t, acc_err, label=f"Acc RMSE {rmse_acc:.3f} m/s$^2$")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error magnitude")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(f"{dataset} Task 6 RMSE — {method} ({frame} frame)")
    fig.tight_layout()

    run_id = f"{dataset}_{method}"
    pdf_path = out_dir / f"{run_id}_Task6_{frame}_RMSE.pdf"
    png_path = out_dir / f"{run_id}_Task6_{frame}_RMSE.png"
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    print(f"[SAVE] {pdf_path}")
    print(f"[SAVE] {png_path}")
    plt.close(fig)
    return pdf_path


# ---------------------------------------------------------------------------
# main entry point
# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Generate Task 6 overlay plot of fused estimate vs truth"
    )
    ap.add_argument("--est-file", required=True, help="fused estimator .npz file")
    ap.add_argument("--truth-file", required=True, help="ground truth STATE_X file")
    ap.add_argument("--method", required=True, help="estimation method name")
    ap.add_argument("--frame", choices=["ECEF", "NED"], default="ECEF", help="reference frame")
    ap.add_argument("--dataset", required=True, help="dataset identifier for filename")
    ap.add_argument("--output-dir", default="results", help="directory for saved figure")
    ap.add_argument("--debug", action="store_true", help="print dataset diagnostics")
    args = ap.parse_args()

    est_path = Path(args.est_file)
    truth_path = Path(args.truth_file)
    out_dir = Path(args.output_dir)

    est = load_estimate(est_path, args.frame)
    if est is None:
        print(f"Missing data in {est_path.name}; overlay skipped.")
        return
    t_est, pos_est, vel_est, acc_est = est
    t_truth, pos_truth, vel_truth, acc_truth = load_truth(truth_path, args.frame)

    if args.debug:
        print_debug_info(t_est, vel_est, t_truth, vel_truth)
        if len(t_est) != len(t_truth):
            print(f"WARNING: time vector lengths differ (est {len(t_est)}, truth {len(t_truth)})")

    pos_truth_i = interpolate_truth(t_est, t_truth, pos_truth)
    vel_truth_i = interpolate_truth(t_est, t_truth, vel_truth)
    acc_truth_i = interpolate_truth(t_est, t_truth, acc_truth)

    (
        t_est,
        pos_est,
        vel_est,
        acc_est,
        pos_truth_i,
        vel_truth_i,
        acc_truth_i,
    ) = ensure_equal_length(
        t_est,
        pos_est,
        vel_est,
        acc_est,
        pos_truth_i,
        vel_truth_i,
        acc_truth_i,
    )

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

    plot_rmse(
        t_est,
        pos_est,
        vel_est,
        acc_est,
        pos_truth_i,
        vel_truth_i,
        acc_truth_i,
        args.frame,
        args.method,
        args.dataset,
        out_dir,
    )

    run_id = f"{args.dataset}_{args.method}"
    run_dir = out_dir / run_id
    saved = sorted(run_dir.glob(f"{run_id}_task6_*.pdf"))
    if saved:
        print("Saved files under", run_dir)
        for f in saved:
            print(" -", f.name)


if __name__ == "__main__":
    main()
