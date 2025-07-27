"""Task 7.1: Investigate Error Between FUSED and Truth.

Usage:
    python src/task7_plot_error_fused_vs_truth.py --fused fused_results.npz \
        --truth truth_results.npz --output-dir results

This script computes the velocity error ``FUSED - Truth`` for each
component and plots the error over time. The resulting figure is saved
as both PDF and PNG under the output directory.

Corresponds to MATLAB/task7_plot_error_fused_vs_truth.m
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_npz(path: Path) -> dict[str, np.ndarray]:
    """Load a NumPy ``.npz`` archive as a dictionary."""
    data = np.load(path)
    return {k: data[k] for k in data.files}


def plot_error(time: np.ndarray, err: np.ndarray, out_dir: Path) -> None:
    """Plot velocity error components and save the figure."""
    labels = ["X", "Y", "Z"]
    plt.figure(figsize=(8, 4))
    for i, lab in enumerate(labels):
        plt.plot(time, err[:, i], label=f"Error {lab}")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity Error [m/s]")
    plt.title("Task 7.1: FUSED - Truth Velocity Error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf = out_dir / "task7_fused_vs_truth_error.pdf"
    png = out_dir / "task7_fused_vs_truth_error.png"
    plt.savefig(pdf)
    plt.savefig(png)
    plt.close()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fused", type=Path, required=True, help="fused .npz file")
    parser.add_argument("--truth", type=Path, required=True, help="truth .npz file")
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    args = parser.parse_args(argv)

    fused = load_npz(args.fused)
    truth = load_npz(args.truth)

    time = fused.get("time")
    if time is None:
        time = fused.get("time_s")
    if time is None:
        raise KeyError("Missing 'time' or 'time_s' in fused file")

    vx = fused.get("vx")
    vy = fused.get("vy")
    vz = fused.get("vz")
    if vx is None or vy is None or vz is None:
        vel = fused.get("vel_ned_ms")
        if vel is None:
            raise KeyError("Missing velocity components in fused file")
        vx, vy, vz = vel[:, 0], vel[:, 1], vel[:, 2]

    tvx = truth.get("vx")
    tvy = truth.get("vy")
    tvz = truth.get("vz")
    if tvx is None or tvy is None or tvz is None:
        vel_t = truth.get("vel_ned_ms")
        if vel_t is None:
            raise KeyError("Missing velocity components in truth file")
        tvx, tvy, tvz = vel_t[:, 0], vel_t[:, 1], vel_t[:, 2]

    err = np.column_stack((vx - tvx, vy - tvy, vz - tvz))

    plot_error(time, err, args.output_dir)


if __name__ == "__main__":
    main()
