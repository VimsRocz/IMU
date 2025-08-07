"""Task 7 – Plot position and velocity residuals in the ECEF frame.

Usage:
    python task7_ecef_residuals_plot.py --est-file <fused.npz> \
        --imu-file <IMU.dat> --gnss-file <GNSS.csv> [--truth-file <STATE_X.txt>] \
        --output-dir results

This script loads a fused estimator output file and the corresponding
ground truth trajectory.  If the estimator output already contains the
``truth_pos_ecef``, ``truth_vel_ecef`` and ``truth_time`` fields produced
by Task 4, the ``--truth-file`` argument is optional.  Otherwise the
STATE_X text log must be provided or inferrable from the dataset name.
The truth trajectory is synchronised to the estimator by cross-correlating
position and velocity magnitudes before interpolation to the estimator time
vector.  Position, velocity and acceleration residuals are plotted.  The time axis is
converted to ``t - t[0]`` so Task 6 and Task 7 share the same reference.
Figures are saved as PDF and PNG under ``results/<tag>/`` where ``tag``
combines the dataset, GNSS file and method.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from validate_with_truth import load_estimate, assemble_frames
from naming import make_tag, plot_filename
import re


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
    gnss: str,
    method: str,
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

    tag = make_tag(dataset, gnss, method)
    fig.suptitle(f"{tag} Task 7 ECEF Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_name = plot_filename(dataset, gnss, method, 7, "3", "ecef_residuals")
    pdf = out_dir / pdf_name
    png = pdf.with_suffix(".png")
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
    fig.suptitle(f"{tag} Task 7 ECEF Residual Norms")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    norm_name = plot_filename(dataset, gnss, method, 7, "3", "ecef_residual_norms")
    norm_pdf = out_dir / norm_name
    norm_png = norm_pdf.with_suffix(".png")
    fig.savefig(norm_pdf)
    fig.savefig(norm_png)
    plt.close(fig)

    saved = sorted(out_dir.glob(f"{tag}_task7_*ecef_residual*.pdf"))
    if saved:
        print("Files saved in", out_dir)
        for f in saved:
            print(" -", f.name)


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot ECEF residuals for Task 7")
    ap.add_argument("--est-file", required=True, help="fused estimator .npz")
    ap.add_argument(
        "--imu-file", required=True, help="raw IMU file used for the estimate"
    )
    ap.add_argument(
        "--gnss-file", required=True, help="raw GNSS file used for the estimate"
    )
    ap.add_argument(
        "--truth-file",
        help=(
            "ground truth STATE_X file. If omitted the script uses truth data "
            "embedded in the estimator output or attempts to infer the path "
            "from --dataset"
        ),
    )
    ap.add_argument("--dataset", required=True, help="IMU dataset file")
    ap.add_argument("--gnss", required=True, help="GNSS dataset file")
    ap.add_argument("--method", required=True, help="initialisation method")
    ap.add_argument("--output-dir", default="results", help="directory for plots")
    args = ap.parse_args()

    est = load_estimate(args.est_file)
    truth_file = args.truth_file
    has_embedded_truth = (
        est.get("truth_pos_ecef") is not None
        and np.asarray(est.get("truth_pos_ecef")).size > 0
    )
    if truth_file is None and not has_embedded_truth:
        m = re.search(r"X(\d+)", Path(args.dataset).stem)
        if m:
            dataset_id = m.group(1)
            candidates = [
                Path(f"STATE_X{dataset_id}.txt"),
                Path(f"STATE_X{dataset_id}_small.txt"),
            ]
            for cand in candidates:
                if cand.is_file():
                    truth_file = str(cand)
                    break
    if truth_file is None and not has_embedded_truth:
        raise FileNotFoundError(
            "Truth file not specified and could not be inferred from --dataset"
        )

    frames = assemble_frames(est, args.imu_file, args.gnss_file, truth_file)
    try:
        t_est, pos_est, vel_est, _ = frames["ECEF"]["fused"]
        truth_tuple = frames["ECEF"].get("truth")
        if truth_tuple is None:
            raise KeyError("truth")
        _, pos_truth, vel_truth, _ = truth_tuple
    except KeyError as exc:
        raise ValueError("Truth data required for ECEF residuals") from exc

    # Align time axis to start at zero for direct comparison with Task 6
    t_rel = t_est - t_est[0]

    res_pos, res_vel, res_acc = compute_residuals(
        t_rel, pos_est, vel_est, pos_truth, vel_truth
    )

    out_dir = Path(args.output_dir)
    plot_residuals(
        t_rel,
        res_pos,
        res_vel,
        res_acc,
        args.dataset,
        args.gnss,
        args.method,
        out_dir,
    )


if __name__ == "__main__":
    main()
