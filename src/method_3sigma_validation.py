"""Compare filter estimates with ground truth and plot error with ±3σ bounds."""

from __future__ import annotations

import argparse
from pathlib import Path
import os

import numpy as np
import matplotlib.pyplot as plt

from validate_with_truth import load_estimate


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Validate filter output against truth with 3-sigma bounds"
    )
    ap.add_argument(
        "--methods",
        nargs="+",
        default=["TRIAD", "Davenport", "SVD"],
        help="Initialisation methods to process",
    )
    ap.add_argument(
        "--dataset",
        default="X001",
        help="Dataset identifier used in result filenames",
    )
    ap.add_argument(
        "--results-dir",
        default="results",
        help="Directory containing *_kf_output.npz files",
    )
    ap.add_argument(
        "--truth",
        default="STATE_X001.txt",
        help="Reference trajectory file",
    )
    ap.add_argument(
        "--output-dir",
        default="results",
        help="Directory to write the validation plots",
    )
    args = ap.parse_args()

    truth = np.loadtxt(args.truth, comments="#")
    t_truth = truth[:, 1]
    pos_truth = truth[:, 2:5]
    vel_truth = truth[:, 5:8]

    Path(args.output_dir).mkdir(parents=True, exist_ok=True)

    for method in args.methods:
        tag = f"IMU_{args.dataset}_GNSS_{args.dataset}_{method}"
        est_file = Path(args.results_dir) / f"{tag}_kf_output.npz"
        if not est_file.exists():
            print(f"Warning: {est_file} not found, skipping {method}")
            continue

        est = load_estimate(str(est_file), times=t_truth)
        pos_est = np.asarray(est.get("pos"))
        vel_est = np.asarray(est.get("vel"))
        P = est.get("P")

        pos_err = pos_est - pos_truth[: len(pos_est)]
        vel_err = vel_est - vel_truth[: len(vel_est)]

        sigma_pos = sigma_vel = None
        if P is not None:
            diag = np.diagonal(P, axis1=1, axis2=2)
            n = min(len(pos_err), diag.shape[0])
            diag = diag[:n]
            sigma_pos = 3 * np.sqrt(diag[:, :3])
            if diag.shape[1] >= 6:
                sigma_vel = 3 * np.sqrt(diag[:, 3:6])

        fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
        comps = ["X", "Y", "Z"]
        for i, comp in enumerate(comps):
            axes[0, i].plot(t_truth[: len(pos_err)], pos_err[:, i], label="error")
            if sigma_pos is not None:
                axes[0, i].plot(t_truth[: len(pos_err)], sigma_pos[:, i], "r--", label="+3σ")
                axes[0, i].plot(t_truth[: len(pos_err)], -sigma_pos[:, i], "r--")
                exceed = np.abs(pos_err[:, i]) > sigma_pos[:, i]
                axes[0, i].plot(t_truth[: len(pos_err)][exceed], pos_err[:, i][exceed], "ro", ms=2)
            axes[0, i].set_ylabel(f"Δ{comp} [m]")
            axes[0, i].grid(True)
            axes[1, i].plot(t_truth[: len(vel_err)], vel_err[:, i], label="error")
            if sigma_vel is not None:
                axes[1, i].plot(t_truth[: len(vel_err)], sigma_vel[:, i], "r--", label="+3σ")
                axes[1, i].plot(t_truth[: len(vel_err)], -sigma_vel[:, i], "r--")
                exceed = np.abs(vel_err[:, i]) > sigma_vel[:, i]
                axes[1, i].plot(t_truth[: len(vel_err)][exceed], vel_err[:, i][exceed], "ro", ms=2)
            axes[1, i].set_ylabel(f"ΔV{comp} [m/s]")
            axes[1, i].grid(True)
        axes[1, 1].set_xlabel("Time [s]")
        axes[0, 0].legend(loc="upper right")
        fig.suptitle(f"{method} Error vs. 3σ Bounds")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        out = Path(args.output_dir) / f"{method}_3sigma_validation.pdf"
        fig.savefig(out)
        plt.close(fig)

        if sigma_pos is not None:
            viol_pos = np.sum(np.any(np.abs(pos_err) > sigma_pos, axis=1))
        else:
            viol_pos = 0
        if sigma_vel is not None:
            viol_vel = np.sum(np.any(np.abs(vel_err) > sigma_vel, axis=1))
        else:
            viol_vel = 0
        total = len(pos_err)
        print(f"{method}: pos {viol_pos}/{total} samples beyond 3σ, "
              f"vel {viol_vel}/{total} samples beyond 3σ")


if __name__ == "__main__":
    main()

