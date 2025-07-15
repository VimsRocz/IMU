#!/usr/bin/env python3
"""Compare filter errors with 3-sigma bounds.

This utility supports both ``.mat`` and ``.npz`` estimator files.  The
variable names are normalised so that different pipelines can be used
interchangeably.  Ground truth is provided in ``STATE_X001.txt``
with columns ``[time,x,y,z,vx,vy,vz,qw,qx,qy,qz]``.  Only the overlapping
time window is used when computing the error and the ±3σ envelopes.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import logging

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import matplotlib.pyplot as plt

# Reuse the robust loader from validate_with_truth
from validate_with_truth import load_estimate

logging.basicConfig(level=logging.INFO, format="%(message)s")


def load_truth(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return time, position, velocity and quaternion arrays.

    The ground-truth files normally contain a leading ``count`` column followed
    by ``time, x, y, z, vx, vy, vz, q0, q1, q2, q3``.  Older files may omit the
    count column.  This loader transparently handles both formats.
    """

    data = np.loadtxt(path)

    if data.ndim != 2 or data.shape[1] < 11:
        raise ValueError(f"Unexpected truth data shape {data.shape}")

    # Detect optional leading count column (12 columns instead of 11)
    offset = 1 if data.shape[1] >= 12 else 0

    time = data[:, offset + 0]
    pos = data[:, offset + 1 : offset + 4]
    vel = data[:, offset + 4 : offset + 7]
    quat = data[:, offset + 7 : offset + 11]
    return time, pos, vel, quat


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Validate estimate against truth using 3-sigma bounds"
    )
    ap.add_argument("--est-file", required=True, help="Path to .mat or .npz estimate file")
    ap.add_argument(
        "--truth-file",
        default="STATE_X001.txt",
        help="Path to STATE_X001.txt ground truth",
    )
    ap.add_argument(
        "--output-dir", default="results/", help="Directory where plots are saved"
    )
    args = ap.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    logging.info("Ensured '%s' directory exists.", out_dir)

    est = load_estimate(args.est_file)

    t = np.asarray(est.get("time")).squeeze()
    pos = np.asarray(est.get("pos"))
    vel = np.asarray(est.get("vel"))
    quat = np.asarray(est.get("quat"))
    P = np.asarray(est.get("P")) if est.get("P") is not None else None

    truth_t, truth_pos, truth_vel, truth_quat = load_truth(args.truth_file)

    # clip truth to the estimator time window
    mask = (truth_t >= t.min()) & (truth_t <= t.max())
    truth_t = truth_t[mask]
    truth_pos = truth_pos[mask]
    truth_vel = truth_vel[mask]
    truth_quat = truth_quat[mask]

    pos_truth_i = np.vstack(
        [np.interp(t, truth_t, truth_pos[:, i]) for i in range(3)]
    ).T
    vel_truth_i = np.vstack(
        [np.interp(t, truth_t, truth_vel[:, i]) for i in range(3)]
    ).T
    # quaternion interpolation via spherical linear interpolation
    r_truth = R.from_quat(truth_quat[:, [1, 2, 3, 0]])
    slerp = Slerp(truth_t, r_truth)
    quat_truth_i = slerp(t).as_quat()[:, [3, 0, 1, 2]]

    err_pos = pos - pos_truth_i
    err_vel = vel - vel_truth_i
    err_quat = quat - quat_truth_i
    r_est = R.from_quat(quat[:, [1, 2, 3, 0]])
    r_truth_i = R.from_quat(quat_truth_i[:, [1, 2, 3, 0]])
    euler_err = (r_truth_i.inv() * r_est).as_euler("xyz", degrees=True)

    rmse_pos = float(np.sqrt(np.mean(np.sum(err_pos**2, axis=1))))
    rmse_vel = float(np.sqrt(np.mean(np.sum(err_vel**2, axis=1))))
    rmse_att = float(np.sqrt(np.mean(np.sum(euler_err**2, axis=1))))
    final_pos = float(np.linalg.norm(err_pos[-1]))
    final_vel = float(np.linalg.norm(err_vel[-1]))
    final_att = float(np.linalg.norm(euler_err[-1]))

    if P is not None:
        diag = np.diagonal(P, axis1=1, axis2=2)
        sigma_pos = 3 * np.sqrt(diag[:, 0:3])
        sigma_vel = 3 * np.sqrt(diag[:, 3:6])
        sigma_quat = 3 * np.sqrt(diag[:, -4:]) if diag.shape[1] >= 7 else None
    else:
        sigma_pos = sigma_vel = sigma_quat = None

    plt.rcParams["axes.grid"] = True

    labels_pos = ["X", "Y", "Z"]
    labels_vel = ["Vx", "Vy", "Vz"]
    labels_quat = ["q0", "q1", "q2", "q3"]

    for i, lab in enumerate(labels_pos):
        fig, ax = plt.subplots()
        ax.plot(t, err_pos[:, i], label="error")
        if sigma_pos is not None:
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
        if sigma_vel is not None:
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
        if sigma_quat is not None:
            ax.plot(t, sigma_quat[:, i], "r--", label="+3σ")
            ax.plot(t, -sigma_quat[:, i], "r--")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"Quaternion {lab} error")
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f"quat_err_{lab}.pdf")
        plt.close(fig)

    print(
        f"Final position error: {final_pos:.3f} m, RMSE: {rmse_pos:.3f} m"
    )
    print(
        f"Final velocity error: {final_vel:.3f} m/s, RMSE: {rmse_vel:.3f} m/s"
    )
    print(
        f"Final attitude error: {final_att:.3f} deg, RMSE: {rmse_att:.3f} deg"
    )


if __name__ == "__main__":
    main()
