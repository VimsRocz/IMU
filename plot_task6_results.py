"""Plot Task 6 fused results from a ``.mat`` file.

This script expects a fused GNSS/IMU output file generated by the
estimator. Position and velocity time series must be provided in the
NED, ECEF and body frames using the variable names ``pos_*`` and
``vel_*`` along with ``att_quat`` and ``time_s``. Figures are saved
under ``results/`` using ``<dataset>_<method>_task6_<type>.pdf`` and
``.png`` filenames.

Usage:
    python plot_task6_results.py results/IMU_X001_GNSS_X001_Davenport.mat
"""

import argparse
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio

# Ensure results directory exists
os.makedirs("results", exist_ok=True)


def load_data(mat_path: Path) -> dict:
    """Load MATLAB ``.mat`` file and return the data dictionary."""

    if not mat_path.exists():
        raise FileNotFoundError(mat_path)
    return sio.loadmat(mat_path)


def parse_run_id(stem: str) -> tuple[str, str]:
    """Return dataset and method from a file stem."""

    parts = stem.split("_")
    if len(parts) < 2:
        return stem, "METHOD"
    return "_".join(parts[:-1]), parts[-1]


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot Task 6 fused results")
    ap.add_argument("mat_file", help="MAT file produced by the estimator")
    ap.add_argument("--output-dir", default="results", help="directory for figures")
    args = ap.parse_args()

    mat_path = Path(args.mat_file)
    data = load_data(mat_path)

    dataset, method = parse_run_id(mat_path.stem)

    print(f"Available keys: {list(data.keys())}")

    try:
        pos_ned = data["pos_ned_m"]
        vel_ned = data["vel_ned_ms"]
        att_quat = data["att_quat"]
        time_s = data["time_s"].squeeze()
        pos_ecef = data["pos_ecef_m"]
        vel_ecef = data["vel_ecef_ms"]
        pos_body = data["pos_body_m"]
        vel_body = data["vel_body_ms"]
    except KeyError as exc:
        print(f"KeyError: {exc}. Check data keys.")
        raise

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    plot_task6_results(
        pos_ned,
        vel_ned,
        att_quat,
        time_s,
        pos_ecef,
        vel_ecef,
        pos_body,
        vel_body,
        dataset,
        method,
        out_dir,
    )


if __name__ == "__main__":
    main()




def plot_task6_results(
    pos_ned,
    vel_ned,
    att_quat,
    time_s,
    pos_ecef,
    vel_ecef,
    pos_body,
    vel_body,
    dataset: str,
    method: str,
    out_dir: Path = Path("results"),
):
    """Generate Task 6 plots and save them under ``out_dir``."""

    run_id = f"{dataset}_{method}"

    # NED frame plots
    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["North", "East", "Down"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, pos_ned[:, i], label=label)
        plt.title(f"{method} Position {label} (m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
    plt.tight_layout()
    base = out_dir / f"{run_id}_task6_position_ned"
    plt.savefig(base.with_suffix(".pdf"))
    plt.savefig(base.with_suffix(".png"))
    plt.close()

    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["North", "East", "Down"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, vel_ned[:, i], label=label)
        plt.title(f"{method} Velocity {label} (m/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
    plt.tight_layout()
    base = out_dir / f"{run_id}_task6_velocity_ned"
    plt.savefig(base.with_suffix(".pdf"))
    plt.savefig(base.with_suffix(".png"))
    plt.close()

    # ECEF frame plots
    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["X", "Y", "Z"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, pos_ecef[:, i], label=label)
        plt.title(f"{method} Position {label} (ECEF, m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
    plt.tight_layout()
    base = out_dir / f"{run_id}_task6_position_ecef"
    plt.savefig(base.with_suffix(".pdf"))
    plt.savefig(base.with_suffix(".png"))
    plt.close()

    # Body frame plots
    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["X", "Y", "Z"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, pos_body[:, i], label=label)
        plt.title(f"{method} Position {label} (Body, m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
    plt.tight_layout()
    base = out_dir / f"{run_id}_task6_position_body"
    plt.savefig(base.with_suffix(".pdf"))
    plt.savefig(base.with_suffix(".png"))
    plt.close()

    # Log results
    print(
        f"Subtask 6.8.2: Plotted {method} position North: First = {pos_ned[0, 0]:.4f}, Last = {pos_ned[-1, 0]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} position East: First = {pos_ned[0, 1]:.4f}, Last = {pos_ned[-1, 1]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} position Down: First = {pos_ned[0, 2]:.4f}, Last = {pos_ned[-1, 2]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} velocity North: First = {vel_ned[0, 0]:.4f}, Last = {vel_ned[-1, 0]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} velocity East: First = {vel_ned[0, 1]:.4f}, Last = {vel_ned[-1, 1]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} velocity Down: First = {vel_ned[0, 2]:.4f}, Last = {vel_ned[-1, 2]:.4f}"
    )
    print(f"Subtask 6.8.2: {run_id} plots saved under {out_dir}")

