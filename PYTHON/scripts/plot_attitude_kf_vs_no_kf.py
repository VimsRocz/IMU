#!/usr/bin/env python3
"""Compare attitude (quaternions/Euler) with and without Kalman filter.

This script plots two figures:
- Attitude with Kalman filter: truth vs estimator quaternion (if available)
- Attitude without Kalman filter: truth vs dead-reckoned gyro integration

Optionally initializes the dead-reckoned attitude with the true initial
quaternion to isolate initialization issues.

Usage:
  python scripts/plot_attitude_kf_vs_no_kf.py \
      --imu-file IMU_X001.dat \
      --est-file results/IMU_X001_GNSS_X001_TRIAD_kf_output.npz \
      --truth-file <path>/STATE_X001.txt \
      [--true-init] [--static-window 400] [--out-dir results]
"""

from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import sys
import os
from pathlib import Path as _Path

# Ensure the project root (parent of this scripts dir) is on sys.path so that
# `import src...` works when running this file directly.
_HERE = _Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.validate_with_truth import load_estimate


def load_truth(path: Path):
    data = np.loadtxt(path)
    t = data[:, 1]
    quat = data[:, 8:12]  # [w x y z]
    return t, quat


def quat_mult(q, r):
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    ])


def quat_from_rate(omega: np.ndarray, dt: float) -> np.ndarray:
    theta = float(np.linalg.norm(omega) * dt)
    if theta == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    half = theta * 0.5
    return np.array([np.cos(half), *(np.sin(half) * axis)])


def integrate_gyro(time: np.ndarray, gyro_rate: np.ndarray, q0: np.ndarray) -> np.ndarray:
    q_hist = np.zeros((len(time), 4))
    q = q0.copy()
    q_hist[0] = q
    for i in range(1, len(time)):
        dt = time[i] - time[i-1]
        dq = quat_from_rate(gyro_rate[i], dt)
        q = quat_mult(q, dq)
        q = q / np.linalg.norm(q)
        q_hist[i] = q
    return q_hist


def to_euler_deg(quat_wxyz: np.ndarray) -> np.ndarray:
    # scipy expects [x y z w]
    rot = R.from_quat(quat_wxyz[:, [1, 2, 3, 0]])
    return rot.as_euler("xyz", degrees=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--imu-file", required=True)
    ap.add_argument("--est-file", required=True)
    ap.add_argument("--truth-file", required=True)
    ap.add_argument("--static-window", type=int, default=400)
    ap.add_argument("--true-init", action="store_true",
                    help="Initialize dead-reckoning with true initial quaternion")
    ap.add_argument("--out-dir", default="results")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load truth
    t_truth, quat_truth = load_truth(Path(args.truth_file))

    # Load estimator
    est = load_estimate(args.est_file)
    t_est = np.asarray(est.get("time")).squeeze()
    quat_est = est.get("quat")
    if quat_est is None and est.get("euler") is not None:
        # synthesize from euler if only angles stored
        quat_xyzw = R.from_euler("xyz", est["euler"], degrees=True).as_quat()
        quat_est = quat_xyzw[:, [3, 0, 1, 2]]
    quat_est = np.asarray(quat_est) if quat_est is not None else None
    # Ensure estimator time and quaternion histories have equal length
    if quat_est is not None and t_est is not None:
        n = min(len(t_est), len(quat_est))
        if len(t_est) != len(quat_est):
            print(f"[WARN] est time/quat length mismatch: {len(t_est)} vs {len(quat_est)}; trimming to {n}")
        t_est = t_est[:n]
        quat_est = quat_est[:n]

    # Load IMU for dead-reckoning (no KF)
    imu = np.loadtxt(args.imu_file)
    # Use provided time if present, else uniform step from first 200 samples
    if imu.shape[1] > 1:
        t_imu = imu[:, 1]
        # Some IMU files use 0-based or arbitrary time; if non-monotone, rebuild
        if not np.all(np.diff(t_imu) > 0):
            dt = np.mean(np.diff(imu[:200, 1]))
            t_imu = np.arange(len(imu)) * dt
    else:
        # fallback assuming 400 Hz
        dt = 1.0 / 400.0
        t_imu = np.arange(len(imu)) * dt
    dt_imu = float(np.median(np.diff(t_imu)))

    # Gyro increments are typically columns 2:5; convert to rad/s
    gyro_inc = imu[:, 2:5]
    gyro_rate = gyro_inc / dt_imu

    # Estimate static gyro bias from first window
    win = min(args.static_window, len(gyro_rate))
    gyro_bias = gyro_rate[:win].mean(axis=0)
    gyro_rate_corr = gyro_rate - gyro_bias

    # Initial quaternion for dead-reckoning
    if args.true_init:
        # Interpolate truth quaternion to IMU t0
        r_truth = R.from_quat(quat_truth[:, [1, 2, 3, 0]])
        # Find nearest truth index to IMU start
        i0 = np.argmin(np.abs(t_truth - t_imu[0]))
        q0 = quat_truth[i0]
    elif quat_est is not None and len(quat_est) > 0:
        q0 = quat_est[0]
    else:
        # Fallback: identity
        q0 = np.array([1.0, 0.0, 0.0, 0.0])

    # Dead-reckoned quaternion from gyro only
    quat_dr = integrate_gyro(t_imu, gyro_rate_corr, q0)

    # Align time vectors (simple clamp to common range)
    t0 = max(t_imu[0], t_est[0] if t_est is not None else t_imu[0], t_truth[0])
    t1 = min(t_imu[-1], t_est[-1] if t_est is not None else t_imu[-1], t_truth[-1])
    def clip(t, arr):
        m = (t >= t0) & (t <= t1)
        return t[m], arr[m]
    t_imu_c, quat_dr_c = clip(t_imu, quat_dr)

    # Interpolate truth and estimator to IMU times for plotting
    def interp_quat(t_src, quat_src, t_dst):
        if quat_src is None:
            return None
        r_src = R.from_quat(quat_src[:, [1, 2, 3, 0]])
        from scipy.spatial.transform import Slerp
        slerp = Slerp(t_src, r_src)
        tq = np.clip(t_dst, t_src[0], t_src[-1])
        return slerp(tq).as_quat()[:, [3, 0, 1, 2]]

    quat_truth_i = interp_quat(t_truth, quat_truth, t_imu_c)
    quat_est_i = interp_quat(t_est, quat_est, t_imu_c) if quat_est is not None else None

    eul_truth = to_euler_deg(quat_truth_i)
    eul_dr = to_euler_deg(quat_dr_c)
    eul_est = to_euler_deg(quat_est_i) if quat_est_i is not None else None

    # Plot: with Kalman filter (truth vs est)
    if eul_est is not None:
        fig, ax = plt.subplots(3, 1, sharex=True)
        labels = ["roll", "pitch", "yaw"]
        for i in range(3):
            ax[i].plot(t_imu_c, eul_est[:, i], label="est (KF)")
            ax[i].plot(t_imu_c, eul_truth[:, i], label="truth", alpha=0.8)
            ax[i].set_ylabel(f"{labels[i]} [deg]")
            ax[i].grid(True)
            ax[i].legend()
        ax[-1].set_xlabel("Time [s]")
        fig.suptitle("Attitude: Kalman Filter vs Truth")
        fig.tight_layout()
        p = out_dir / (Path(args.est_file).stem + "_att_kf_vs_truth.png")
        fig.savefig(p)
        plt.close(fig)
        print("Saved:", p)

    # Plot: without Kalman filter (truth vs dead-reckoned gyro)
    fig, ax = plt.subplots(3, 1, sharex=True)
    labels = ["roll", "pitch", "yaw"]
    for i in range(3):
        ax[i].plot(t_imu_c, eul_dr[:, i], label="gyro DR (no KF)")
        ax[i].plot(t_imu_c, eul_truth[:, i], label="truth", alpha=0.8)
        ax[i].set_ylabel(f"{labels[i]} [deg]")
        ax[i].grid(True)
        ax[i].legend()
    ax[-1].set_xlabel("Time [s]")
    ttl = "Attitude: Gyro Dead-Reckoning vs Truth"
    if args.true_init:
        ttl += " (true init)"
    fig.suptitle(ttl)
    fig.tight_layout()
    p2 = out_dir / (Path(args.est_file).stem + ("_att_dr_trueinit.png" if args.true_init else "_att_dr.png"))
    fig.savefig(p2)
    plt.close(fig)
    print("Saved:", p2)


if __name__ == "__main__":
    main()
