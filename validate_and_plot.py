#!/usr/bin/env python3
"""Compare estimator output against ground truth and plot results.

Usage: python validate_and_plot.py --est-file <est-file> --truth-file <truth-file> --output-dir results
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp

from src.validate_with_truth import load_estimate
from src.utils import compute_C_ECEF_to_NED, ecef_to_geodetic


def load_truth(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return time, ECEF position, velocity and quaternion from ``STATE_X*.txt``."""
    data = np.loadtxt(path)
    t = data[:, 1]
    pos = data[:, 2:5]
    vel = data[:, 5:8]
    quat = data[:, 8:12]
    return t, pos, vel, quat


def to_ecef(pos_ned: np.ndarray, vel_ned: np.ndarray, ref_lat: float, ref_lon: float, ref_ecef: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert NED position/velocity to ECEF."""
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    pos_ecef = (C.T @ pos_ned.T).T + ref_ecef
    vel_ecef = (C.T @ vel_ned.T).T
    return pos_ecef, vel_ecef


def align_truth(t_est: np.ndarray, t_truth: np.ndarray, pos_truth: np.ndarray, vel_truth: np.ndarray, quat_truth: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate truth position, velocity and quaternion to estimator times."""
    pos_i = np.vstack([np.interp(t_est, t_truth, pos_truth[:, i]) for i in range(3)]).T
    vel_i = np.vstack([np.interp(t_est, t_truth, vel_truth[:, i]) for i in range(3)]).T
    r_truth = R.from_quat(quat_truth[:, [1, 2, 3, 0]])
    slerp = Slerp(t_truth, r_truth)
    quat_i = slerp(np.clip(t_est, t_truth[0], t_truth[-1])).as_quat()[:, [3, 0, 1, 2]]
    return pos_i, vel_i, quat_i


def compute_stats(res: np.ndarray) -> dict[str, float]:
    """Return RMSE, final error, mean, std and max error for a residual array."""
    norm = np.linalg.norm(res, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(norm**2))),
        "final": float(norm[-1]),
        "mean": float(np.mean(norm)),
        "std": float(np.std(norm)),
        "max": float(np.max(norm)),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--est-file", required=True, help="state estimator output (.mat or .npz)")
    ap.add_argument("--truth-file", required=True, help="ground truth STATE_X*.txt")
    ap.add_argument("--output-dir", default="results", help="directory for saved plots")
    args = ap.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    t_truth, pos_truth_ecef, vel_truth_ecef, quat_truth = load_truth(Path(args.truth_file))

    est = load_estimate(args.est_file)
    t_est = np.asarray(est["time"]).squeeze()
    pos_est = np.asarray(est["pos"])
    vel_est = np.asarray(est["vel"])
    quat_est = np.asarray(est.get("quat"))

    ref_lat = est.get("ref_lat") or est.get("ref_lat_rad") or est.get("lat0")
    ref_lon = est.get("ref_lon") or est.get("ref_lon_rad") or est.get("lon0")
    ref_ecef = est.get("ref_r0") or est.get("ref_r0_m") or est.get("r0")

    if ref_lat is None or ref_lon is None or ref_ecef is None:
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_truth_ecef[0])
        ref_lat = np.deg2rad(lat_deg)
        ref_lon = np.deg2rad(lon_deg)
        ref_ecef = pos_truth_ecef[0]
    else:
        ref_lat = float(np.asarray(ref_lat).squeeze())
        ref_lon = float(np.asarray(ref_lon).squeeze())
        ref_ecef = np.asarray(ref_ecef).squeeze()

    pos_est_ecef, vel_est_ecef = to_ecef(pos_est, vel_est, ref_lat, ref_lon, ref_ecef)

    # restrict to common time range
    mask = (t_est >= t_truth[0]) & (t_est <= t_truth[-1])
    t_est = t_est[mask]
    pos_est_ecef = pos_est_ecef[mask]
    vel_est_ecef = vel_est_ecef[mask]
    quat_est = quat_est[mask] if quat_est is not None else None

    pos_truth_i, vel_truth_i, quat_truth_i = align_truth(t_est, t_truth, pos_truth_ecef, vel_truth_ecef, quat_truth)

    pos_res = pos_est_ecef - pos_truth_i
    vel_res = vel_est_ecef - vel_truth_i
    if quat_est is not None:
        r_est = R.from_quat(quat_est[:, [1, 2, 3, 0]])
        r_truth_i = R.from_quat(quat_truth_i[:, [1, 2, 3, 0]])
        att_res = (r_truth_i.inv() * r_est).as_euler("xyz", degrees=True)
    else:
        att_res = np.zeros_like(pos_res)

    stats_pos = compute_stats(pos_res)
    stats_vel = compute_stats(vel_res)
    stats_att = compute_stats(att_res)

    m = re.search(r"(IMU_\w+).*_([A-Za-z]+)_kf_output", Path(args.est_file).stem)
    dataset = m.group(1) if m else Path(args.est_file).stem
    method = m.group(2) if m else "est"

    # -- plotting ------------------------------------------------------------
    plot_paths: list[Path] = []

    # 3D trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pos_truth_ecef[:, 0], pos_truth_ecef[:, 1], pos_truth_ecef[:, 2], label="truth")
    ax.plot(pos_est_ecef[:, 0], pos_est_ecef[:, 1], pos_est_ecef[:, 2], label="estimate")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("ECEF Trajectory")
    ax.legend()
    ax.grid(True)
    path = out_dir / f"{dataset}_{method}_traj3d.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    plot_paths.append(path)

    # X/Y/Z vs time
    fig, axs = plt.subplots(3, 1, sharex=True)
    labels = ["X", "Y", "Z"]
    for i in range(3):
        axs[i].plot(t_est, pos_est_ecef[:, i], label="est")
        axs[i].plot(t_est, pos_truth_i[:, i], label="truth")
        axs[i].set_ylabel(f"{labels[i]} [m]")
        axs[i].grid(True)
    axs[0].set_title("ECEF position vs time")
    axs[-1].set_xlabel("Time [s]")
    axs[0].legend()
    path = out_dir / f"{dataset}_{method}_pos_time.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    plot_paths.append(path)

    # residuals
    def plot_residual(res: np.ndarray, ylabel: str, fname: str, units: str = ""):
        fig, axs = plt.subplots(3, 1, sharex=True)
        for i, ax in enumerate(axs):
            ax.plot(t_est, res[:, i])
            ax.set_ylabel(f"{ylabel}{i + 1} [{units}]")
            ax.grid(True)
        axs[-1].set_xlabel("Time [s]")
        axs[0].set_title(f"{ylabel} residuals")
        p = out_dir / fname
        fig.tight_layout()
        fig.savefig(p)
        plt.close(fig)
        plot_paths.append(p)

    plot_residual(pos_res, "Pos", f"{dataset}_{method}_pos_res.png", "m")
    plot_residual(vel_res, "Vel", f"{dataset}_{method}_vel_res.png", "m/s")
    plot_residual(att_res, "Att", f"{dataset}_{method}_att_res.png", "deg")

    # error norms
    fig, ax = plt.subplots()
    ax.plot(t_est, np.linalg.norm(pos_res, axis=1), label="pos [m]")
    ax.plot(t_est, np.linalg.norm(vel_res, axis=1), label="vel [m/s]")
    ax.plot(t_est, np.linalg.norm(att_res, axis=1), label="att [deg]")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error magnitude")
    ax.grid(True)
    ax.legend()
    ax.set_title("Error norm vs time")
    path = out_dir / f"{dataset}_{method}_err_norm.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    plot_paths.append(path)

    # histograms of residuals
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    axs[0].hist(np.linalg.norm(pos_res, axis=1), bins=40)
    axs[0].set_xlabel("|pos residual| [m]")
    axs[0].set_ylabel("Count")
    axs[0].grid(True)
    axs[1].hist(np.linalg.norm(vel_res, axis=1), bins=40)
    axs[1].set_xlabel("|vel residual| [m/s]")
    axs[1].grid(True)
    fig.suptitle("Residual histograms")
    path = out_dir / f"{dataset}_{method}_hist.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    plot_paths.append(path)

    # -- print summary -------------------------------------------------------
    def fmt(stats):
        return (
            f"RMSE={stats['rmse']:.3f}, final={stats['final']:.3f}, "
            f"mean={stats['mean']:.3f}, std={stats['std']:.3f}, max={stats['max']:.3f}"
        )

    print(f"Position error:   {fmt(stats_pos)}")
    print(f"Velocity error:   {fmt(stats_vel)}")
    print(f"Attitude error:   {fmt(stats_att)}")
    print("Saved plots:")
    for p in plot_paths:
        print(" -", p)


if __name__ == "__main__":
    main()
