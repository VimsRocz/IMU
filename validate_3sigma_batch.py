#!/usr/bin/env python3
"""Batch 3-sigma validation for multiple Kalman filter outputs.

This script searches a directory for ``*_kf_output.npz`` files and compares each
estimate against ``STATE_X001.txt``.  A summary of the final and RMS errors for
position, velocity and attitude is written to ``results/validation_summary.csv``.
The logic mirrors ``src/validate_3sigma.py`` but automatically aligns the truth
time vector when it starts at zero.
"""

from __future__ import annotations

import argparse
import csv
import logging
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from tabulate import tabulate

# Reuse the robust estimate loader
from src.validate_with_truth import load_estimate

logging.basicConfig(level=logging.INFO, format="%(message)s")


def load_truth(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return time, position, velocity and quaternion arrays from ``STATE_X001.txt``."""

    data = np.loadtxt(path)
    if data.ndim != 2 or data.shape[1] < 11:
        raise ValueError(f"Unexpected truth data shape {data.shape}")

    offset = 1 if data.shape[1] >= 12 else 0
    t = data[:, offset + 0]
    pos = data[:, offset + 1 : offset + 4]
    vel = data[:, offset + 4 : offset + 7]
    quat = data[:, offset + 7 : offset + 11]
    return t, pos, vel, quat


def validate_file(est_file: Path, truth_file: Path) -> dict[str, float] | None:
    """Validate a single estimate file.

    Parameters
    ----------
    est_file : Path
        ``*_kf_output.npz`` file to load.
    truth_file : Path
        Path to ``STATE_X001.txt``.

    Returns
    -------
    dict or None
        Dictionary with summary statistics or ``None`` when validation failed.
    """

    try:
        est = load_estimate(str(est_file))
    except Exception as exc:  # pragma: no cover - loader errors
        logging.warning("Failed to load %s: %s", est_file, exc)
        return None

    t_est = np.asarray(est.get("time")).squeeze()
    pos_est = np.asarray(est.get("pos"))
    vel_est = np.asarray(est.get("vel"))
    quat_est = np.asarray(est.get("quat"))

    if t_est.size == 0:
        logging.warning("%s has no time vector", est_file)
        return None

    try:
        t_truth, pos_truth, vel_truth, quat_truth = load_truth(truth_file)
    except Exception as exc:  # pragma: no cover - load error
        logging.warning("Failed to load truth %s: %s", truth_file, exc)
        return None

    # Auto-shift truth when it starts at zero and appears to be relative time
    if t_truth.min() >= 0.0 and t_truth.max() < 1e5 and abs(t_truth[0]) < 1e-3:
        shift = t_est[0] - t_truth[0]
        logging.info("Shifting truth by %.3f s to align with estimate", shift)
        t_truth = t_truth + shift

    # Trim truth to estimator window
    mask = (t_truth >= t_est.min()) & (t_truth <= t_est.max())
    if not np.any(mask):
        logging.warning(
            "No overlap between %s and truth (%.2f-%.2f vs %.2f-%.2f)",
            est_file.name,
            t_est[0],
            t_est[-1],
            t_truth[0],
            t_truth[-1],
        )
        return None

    t_truth = t_truth[mask]
    pos_truth = pos_truth[mask]
    vel_truth = vel_truth[mask]
    quat_truth = quat_truth[mask]

    # Interpolate truth to estimate times
    pos_truth_i = np.vstack([np.interp(t_est, t_truth, pos_truth[:, i]) for i in range(3)]).T
    vel_truth_i = np.vstack([np.interp(t_est, t_truth, vel_truth[:, i]) for i in range(3)]).T
    r_truth = R.from_quat(quat_truth[:, [1, 2, 3, 0]])
    slerp = Slerp(t_truth, r_truth)
    quat_truth_i = slerp(t_est).as_quat()[:, [3, 0, 1, 2]]

    err_pos = pos_est - pos_truth_i
    err_vel = vel_est - vel_truth_i
    r_est = R.from_quat(quat_est[:, [1, 2, 3, 0]])
    r_truth_i = R.from_quat(quat_truth_i[:, [1, 2, 3, 0]])
    eul_err = (r_truth_i.inv() * r_est).as_euler("xyz", degrees=True)

    rmse_pos = float(np.sqrt(np.mean(np.sum(err_pos**2, axis=1))))
    rmse_vel = float(np.sqrt(np.mean(np.sum(err_vel**2, axis=1))))
    rmse_att = float(np.sqrt(np.mean(np.sum(eul_err**2, axis=1))))

    final_pos = float(np.linalg.norm(err_pos[-1]))
    final_vel = float(np.linalg.norm(err_vel[-1]))
    final_att = float(np.linalg.norm(eul_err[-1]))

    return {
        "file": est_file.name,
        "rmse_pos": rmse_pos,
        "final_pos": final_pos,
        "rmse_vel": rmse_vel,
        "final_vel": final_vel,
        "rmse_att": rmse_att,
        "final_att": final_att,
    }


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--results-dir",
        type=Path,
        default=Path("results"),
        help="Directory containing *_kf_output.npz files",
    )
    ap.add_argument(
        "--truth-file",
        type=Path,
        default=Path("STATE_X001.txt"),
        help="Ground truth trajectory",
    )
    ap.add_argument(
        "--output",
        type=Path,
        default=Path("results"),
        help="Directory to write validation_summary.csv",
    )
    args = ap.parse_args(argv)

    if not args.results_dir.exists():
        logging.error("Results directory '%s' not found", args.results_dir)
        return
    if not args.truth_file.exists():
        logging.error("Truth file '%s' not found", args.truth_file)
        return

    out_dir = args.output
    out_dir.mkdir(parents=True, exist_ok=True)

    summaries: list[dict[str, float]] = []

    for est_file in sorted(args.results_dir.glob("*_kf_output.npz")):
        logging.info("Validating %s", est_file.name)
        res = validate_file(est_file, args.truth_file)
        if res is not None:
            summaries.append(res)

    if not summaries:
        logging.warning("No valid results found")
        return

    csv_path = out_dir / "validation_summary.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "file",
                "rmse_pos",
                "rmse_vel",
                "rmse_att",
                "final_pos",
                "final_vel",
                "final_att",
            ],
        )
        writer.writeheader()
        for row in summaries:
            writer.writerow(row)
    headers = [
        "Estimate File",
        "Pos RMSE [m]",
        "Vel RMSE [m/s]",
        "Att RMSE [deg]",
        "Final Pos [m]",
        "Final Vel [m/s]",
        "Final Att [deg]",
    ]
    table = [
        [
            row["file"],
            f"{row['rmse_pos']:.2f}",
            f"{row['rmse_vel']:.2f}",
            f"{row['rmse_att']:.2f}",
            f"{row['final_pos']:.2f}",
            f"{row['final_vel']:.2f}",
            f"{row['final_att']:.2f}",
        ]
        for row in summaries
    ]
    print("\n" + tabulate(table, headers=headers))
    logging.info("Wrote %s", csv_path)


if __name__ == "__main__":  # pragma: no cover
    main()
