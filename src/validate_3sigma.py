#!/usr/bin/env python3
"""Compare filter errors with 3-sigma bounds.

This script loads Kalman-filter estimates from a ``.mat`` file and a
``STATE_X***.txt`` ground-truth file.  Position, velocity and quaternion
errors are plotted together with ±3σ envelopes derived from the stored
covariance matrices.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt


def load_truth(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return time, position, velocity and quaternion arrays."""
    data = np.loadtxt(path)
    if data.shape[1] == 13:
        time = data[:, 1]
        pos = data[:, 2:5]
        vel = data[:, 5:8]
        quat = data[:, 8:12]
    else:
        time = data[:, 0]
        pos = data[:, 1:4]
        vel = data[:, 4:7]
        quat = data[:, 7:11]
    return time, pos, vel, quat


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Validate estimate against truth using 3-sigma bounds"
    )
    ap.add_argument("--est-file", required=True, help="Path to .mat estimate file")
    ap.add_argument(
        "--truth-file", required=True, help="Path to STATE_X001.txt ground truth"
    )
    ap.add_argument(
        "--output-dir", default="results/", help="Directory where plots are saved"
    )
    args = ap.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    est = loadmat(args.est_file)
    t = np.asarray(est["t"]).squeeze()
    pos = np.asarray(est["pos"])
    vel = np.asarray(est["vel"])
    P = np.asarray(est["P"])
    quat = np.asarray(est["quat"])

    truth_t, truth_pos, truth_vel, truth_quat = load_truth(args.truth_file)

    pos_truth_i = np.vstack(
        [np.interp(t, truth_t, truth_pos[:, i]) for i in range(3)]
    ).T
    vel_truth_i = np.vstack(
        [np.interp(t, truth_t, truth_vel[:, i]) for i in range(3)]
    ).T
    quat_truth_i = np.vstack(
        [np.interp(t, truth_t, truth_quat[:, i]) for i in range(4)]
    ).T

    err_pos = pos - pos_truth_i
    err_vel = vel - vel_truth_i
    err_quat = quat - quat_truth_i

    diag = np.diagonal(P, axis1=1, axis2=2)
    sigma_pos = 3 * np.sqrt(diag[:, 0:3])
    sigma_vel = 3 * np.sqrt(diag[:, 3:6])
    sigma_quat = 3 * np.sqrt(diag[:, 6:10])

    plt.rcParams["axes.grid"] = True

    labels_pos = ["X", "Y", "Z"]
    labels_vel = ["Vx", "Vy", "Vz"]
    labels_quat = ["q0", "q1", "q2", "q3"]

    for i, lab in enumerate(labels_pos):
        fig, ax = plt.subplots()
        ax.plot(t, err_pos[:, i], label="error")
        ax.plot(t, sigma_pos[:, i], "r--", label="+3σ")
        ax.plot(t, -sigma_pos[:, i], "r--")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"Position {lab} error [m]")
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f"pos_err_{lab}.pdf")
        plt.close(fig)

    for i, lab in enumerate(labels_vel):
        fig, ax = plt.subplots()
        ax.plot(t, err_vel[:, i], label="error")
        ax.plot(t, sigma_vel[:, i], "r--", label="+3σ")
        ax.plot(t, -sigma_vel[:, i], "r--")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"Velocity {lab} error [m/s]")
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f"vel_err_{lab}.pdf")
        plt.close(fig)

    for i, lab in enumerate(labels_quat):
        fig, ax = plt.subplots()
        ax.plot(t, err_quat[:, i], label="error")
        ax.plot(t, sigma_quat[:, i], "r--", label="+3σ")
        ax.plot(t, -sigma_quat[:, i], "r--")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"Quaternion {lab} error")
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f"quat_err_{lab}.pdf")
        plt.close(fig)


if __name__ == "__main__":
    main()
