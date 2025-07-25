"""Task 7 – Plot position and velocity residuals in the ECEF frame.

Usage:
    python task7_ecef_residuals_plot.py --est-file <fused.npz> \
        --imu-file <IMU.dat> --gnss-file <GNSS.csv> \
        --truth-file <STATE_X.txt> --output-dir results

This script loads a fused estimator output file and a ground truth
trajectory, interpolates the truth to the estimator time vector and
plots position, velocity and acceleration residuals. Figures are saved
as PDF and PNG under ``results/task7/<dataset>/`` within the chosen
output directory.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from validate_with_truth import load_estimate, assemble_frames



def compute_residuals(
    t_est: np.ndarray,
    est_pos: np.ndarray,
    est_vel: np.ndarray,
    truth_pos: np.ndarray,
    truth_vel: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return position, velocity and acceleration residuals."""
    res_pos = est_pos - truth_pos
    res_vel = est_vel - truth_vel
    acc_est = np.gradient(est_vel, t_est, axis=0)
    acc_truth = np.gradient(truth_vel, t_est, axis=0)
    res_acc = acc_est - acc_truth
    return res_pos, res_vel, res_acc


def plot_residuals(
    t: np.ndarray,
    res_pos: np.ndarray,
    res_vel: np.ndarray,
    res_acc: np.ndarray,
    dataset: str,
    out_dir: Path,
) -> None:
    """Plot residual components and norms."""
    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)

    for i, (arr, ylabel) in enumerate(
        [
            (res_pos, "Position Residual [m]"),
            (res_vel, "Velocity Residual [m/s]"),
            (res_acc, "Acceleration Residual [m/s$^2$]"),
        ]
    ):
        for j in range(3):
            ax = axes[i, j]
            ax.plot(t, arr[:, j])
            if i == 0:
                ax.set_title(labels[j])
            if j == 0:
                ax.set_ylabel(ylabel)
            if i == 2:
                ax.set_xlabel("Time [s]")
            ax.grid(True)

    fig.suptitle(f"{dataset} Task 7 ECEF Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf = out_dir / f"{dataset}_task7_ecef_residuals.pdf"
    png = out_dir / f"{dataset}_task7_ecef_residuals.png"
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)

    # Norm plot
    fig, ax = plt.subplots()
    ax.plot(t, np.linalg.norm(res_pos, axis=1), label="|pos|")
    ax.plot(t, np.linalg.norm(res_vel, axis=1), label="|vel|")
    ax.plot(t, np.linalg.norm(res_acc, axis=1), label="|acc|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Residual Norm")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    norm_pdf = out_dir / f"{dataset}_task7_ecef_residual_norms.pdf"
    norm_png = out_dir / f"{dataset}_task7_ecef_residual_norms.png"
    fig.savefig(norm_pdf)
    fig.savefig(norm_png)
    plt.close(fig)



def main() -> None:
    ap = argparse.ArgumentParser(description="Plot ECEF residuals for Task 7")
    ap.add_argument("--est-file", required=True, help="fused estimator .npz")
    ap.add_argument("--imu-file", required=True, help="raw IMU file used for the estimate")
    ap.add_argument("--gnss-file", required=True, help="raw GNSS file used for the estimate")
    ap.add_argument("--truth-file", required=True, help="ground truth STATE_X file")
    ap.add_argument("--dataset", required=True, help="dataset identifier")
    ap.add_argument("--output-dir", default="results", help="directory for plots")
    args = ap.parse_args()

    est = load_estimate(args.est_file)
    frames = assemble_frames(est, args.imu_file, args.gnss_file, args.truth_file)
    try:
        t_est, pos_est, vel_est, _ = frames["ECEF"]["fused"]
        _, pos_truth, vel_truth, _ = frames["ECEF"]["truth"]
    except KeyError as exc:
        raise ValueError("Truth data required for ECEF residuals") from exc

    res_pos, res_vel, res_acc = compute_residuals(
        t_est, pos_est, vel_est, pos_truth, vel_truth
    )

    out_dir = Path(args.output_dir) / "task7" / args.dataset
    plot_residuals(t_est, res_pos, res_vel, res_acc, args.dataset, out_dir)


if __name__ == "__main__":
    main()
