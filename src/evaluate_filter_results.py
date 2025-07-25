#!/usr/bin/env python3
"""Task 7 – Evaluation of Filter Results.

This module compares the predicted state from the Kalman filter
against the recorded GNSS measurements and visualises the attitude
history. Residual statistics are printed and all plots are saved
under ``results/`` (or the provided output directory).
"""
from __future__ import annotations
from pathlib import Path
import os
from typing import Sequence

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from tabulate import tabulate
import time
from velocity_utils import derive_velocity


def _find_cols(df: pd.DataFrame, options: Sequence[Sequence[str]]) -> Sequence[str]:
    """Return the first column set that matches the DataFrame columns."""
    for cols in options:
        if all(c in df.columns for c in cols):
            return list(cols)
    raise KeyError(f"None of {options!r} found in {list(df.columns)}")


def run_evaluation(
    prediction_file: str,
    gnss_file: str,
    attitude_file: str,
    save_path: str,
    tag: str | None = None,
) -> None:
    """Compute residuals and plot attitude angles.

    Parameters
    ----------
    prediction_file, gnss_file, attitude_file
        CSV files containing the filter prediction, GNSS measurements and
        attitude estimates.
    save_path
        Output directory where figures are saved.
    tag
        Optional dataset tag added as a prefix to the plot filenames.
    """
    out_dir = Path(save_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = f"{tag}_" if tag else ""

    pred = pd.read_csv(prediction_file)
    gnss = pd.read_csv(gnss_file)
    att = pd.read_csv(attitude_file)

    for df in (pred, gnss, att):
        if "time" not in df.columns:
            for cand in ["t", "timestamp", "Time", "Time_s"]:
                if cand in df.columns:
                    df.rename(columns={cand: "time"}, inplace=True)
                    break

    pos_cols_p = _find_cols(
        pred, [["px", "py", "pz"], ["pos_x", "pos_y", "pos_z"], ["x", "y", "z"]]
    )
    vel_cols_p = _find_cols(pred, [["vx", "vy", "vz"], ["vel_x", "vel_y", "vel_z"]])
    pos_cols_g = _find_cols(
        gnss,
        [
            ["x", "y", "z"],
            ["pos_x", "pos_y", "pos_z"],
            ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"],
        ],
    )
    vel_cols_g = _find_cols(
        gnss,
        [
            ["vx", "vy", "vz"],
            ["vel_x", "vel_y", "vel_z"],
            ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"],
        ],
    )
    quat_cols = _find_cols(att, [["qw", "qx", "qy", "qz"], ["q0", "q1", "q2", "q3"]])

    t_pred = pred["time"].to_numpy()

    gnss_pos_interp = np.vstack(
        [np.interp(t_pred, gnss["time"], gnss[c]) for c in pos_cols_g]
    ).T
    gnss_vel_interp = np.vstack(
        [np.interp(t_pred, gnss["time"], gnss[c]) for c in vel_cols_g]
    ).T

    pred_pos = pred[pos_cols_p].to_numpy()
    pred_vel = pred[vel_cols_p].to_numpy()

    res_pos = gnss_pos_interp - pred_pos
    res_vel = gnss_vel_interp - pred_vel

    mean_pos = res_pos.mean(axis=0)
    std_pos = res_pos.std(axis=0)
    mean_vel = res_vel.mean(axis=0)
    std_vel = res_vel.std(axis=0)

    print("Position residual mean [m]:", mean_pos)
    print("Position residual std  [m]:", std_pos)
    print("Velocity residual mean [m/s]:", mean_vel)
    print("Velocity residual std  [m/s]:", std_vel)

    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
    for i in range(3):
        axes[0, i].plot(t_pred, res_pos[:, i])
        axes[0, i].set_title(labels[i])
        axes[0, i].set_ylabel("Pos Residual [m]")
        axes[0, i].grid(True)
        axes[1, i].plot(t_pred, res_vel[:, i])
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].set_ylabel("Vel Residual [m/s]")
        axes[1, i].grid(True)
    fig.suptitle("Task 7 – GNSS - Predicted Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = out_dir / f"{prefix}task7_3_residuals_position_velocity.pdf"
    fig.savefig(out_path)
    print(f"Saved {out_path}")
    plt.close(fig)

    # Histograms of residuals
    for arr, name in [(res_pos, "position"), (res_vel, "velocity")]:
        fig, axes = plt.subplots(1, 3, figsize=(12, 3))
        for i, lab in enumerate(labels):
            axes[i].hist(arr[:, i], bins=40, alpha=0.7)
            axes[i].set_xlabel(f"{lab} Residual")
            axes[i].set_ylabel("Count")
            axes[i].grid(True)
        fig.suptitle(f"Task 7 – Histogram of {name} residuals")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        hist_path = out_dir / f"{prefix}task7_hist_{name}_residuals.pdf"
        fig.savefig(hist_path)
        print(f"Saved {hist_path}")
        plt.close(fig)

    quat = att[quat_cols].to_numpy()
    rot = R.from_quat(quat[:, [1, 2, 3, 0]])  # w,x,y,z -> x,y,z,w
    euler = rot.as_euler("xyz", degrees=True)
    t_att = att["time"].to_numpy()

    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    names = ["Roll", "Pitch", "Yaw"]
    for i in range(3):
        axs[i].plot(t_att, euler[:, i])
        axs[i].set_ylabel(f"{names[i]} [deg]")
        axs[i].grid(True)
    axs[2].set_xlabel("Time [s]")
    fig.suptitle("Task 7 – Attitude Angles")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_dir / f"{prefix}task7_4_attitude_angles_euler.pdf")
    plt.close(fig)


def run_evaluation_npz(npz_file: str, save_path: str, tag: str | None = None) -> None:
    """Evaluate residuals stored in a ``*_kf_output.npz`` file.

    Parameters
    ----------
    npz_file
        File produced by :mod:`GNSS_IMU_Fusion` containing residual arrays.
    save_path
        Directory where the evaluation plots will be written.
    tag
        Optional dataset tag used to prefix the filenames.
    """
    start_time = time.time()
    out_dir = Path(save_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = f"{tag}_" if tag else ""

    data = np.load(npz_file)
    res_pos = data.get("residual_pos")
    res_vel = data.get("residual_vel")
    t = data.get("time_residuals")
    quat = data.get("attitude_q")
    if res_pos is None or res_vel is None or t is None or quat is None:
        raise KeyError("Required residuals not found in NPZ file")

    if len(t) != len(res_pos) or len(t) != len(res_vel):
        n = min(len(t), len(res_pos), len(res_vel))
        print(
            f"Warning: residual arrays and time length mismatch, truncating to {n} samples"
        )
        t = t[:n]
        res_pos = res_pos[:n]
        res_vel = res_vel[:n]
    else:
        n = len(t)
    quat = quat[:n]

    # Reconstruct GNSS position and derive smoother velocity
    fused_time = data.get("time")
    fused_pos = data.get("pos_ned")
    if fused_pos is None:
        fused_pos = data.get("fused_pos")
    fused_vel = data.get("vel_ned")
    if fused_vel is None:
        fused_vel = data.get("fused_vel")
    if fused_time is not None and fused_pos is not None and fused_vel is not None:
        pos_interp = np.vstack(
            [np.interp(t, fused_time, fused_pos[:, i]) for i in range(3)]
        ).T
        vel_interp = np.vstack(
            [np.interp(t, fused_time, fused_vel[:, i]) for i in range(3)]
        ).T
        truth_pos = pos_interp - res_pos
        truth_vel = derive_velocity(t, truth_pos)
        print(f"Final fused_vel_ned: {vel_interp[-1]}")
        print(f"Final truth_vel_ned: {truth_vel[-1]}")
        res_pos = pos_interp - truth_pos
        res_vel = vel_interp - truth_vel

    mean_pos = res_pos.mean(axis=0)
    std_pos = res_pos.std(axis=0)
    mean_vel = res_vel.mean(axis=0)
    std_vel = res_vel.std(axis=0)

    print("Position residual mean [m]:", mean_pos)
    print("Position residual std  [m]:", std_pos)
    print("Velocity residual mean [m/s]:", mean_vel)
    print("Velocity residual std  [m/s]:", std_vel)

    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
    for i in range(3):
        axes[0, i].plot(t, res_pos[:, i])
        axes[0, i].set_title(labels[i])
        axes[0, i].set_ylabel("Pos Residual [m]")
        axes[0, i].grid(True)
        axes[1, i].plot(t, res_vel[:, i])
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].set_ylabel("Vel Residual [m/s]")
        axes[1, i].grid(True)
    fig.suptitle("Task 7 – GNSS - Predicted Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = out_dir / f"{prefix}task7_3_residuals_position_velocity.pdf"
    fig.savefig(out_path)
    print(f"Saved {out_path}")
    plt.close(fig)

    rot = R.from_quat(quat[:, [1, 2, 3, 0]])
    euler = rot.as_euler("xyz", degrees=True)

    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    names = ["Roll", "Pitch", "Yaw"]
    for i in range(3):
        axs[i].plot(t, euler[:, i])
        axs[i].set_ylabel(f"{names[i]} [deg]")
        axs[i].grid(True)
    axs[2].set_xlabel("Time [s]")
    fig.suptitle("Task 7 – Attitude Angles")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    att_path = out_dir / f"{prefix}task7_4_attitude_angles_euler.pdf"
    fig.savefig(att_path)
    print(f"Saved {att_path}")
    plt.close(fig)

    # Error norm plots
    norm_pos = np.linalg.norm(res_pos, axis=1)
    norm_vel = np.linalg.norm(res_vel, axis=1)
    res_acc = np.gradient(res_vel, t, axis=0)
    norm_acc = np.linalg.norm(res_acc, axis=1)

    fig, ax = plt.subplots()
    ax.plot(t, norm_pos, label="|pos error|")
    ax.plot(t, norm_vel, label="|vel error|")
    ax.plot(t, norm_acc, label="|acc error|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error Norm")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    norm_path = out_dir / f"{prefix}task7_3_error_norms.pdf"
    fig.savefig(norm_path)
    print(f"Saved {norm_path}")
    plt.close(fig)

    # Subtask 7.5: difference Truth - Fused
    if fused_time is not None and fused_pos is not None and fused_vel is not None:
        run_id = tag.replace(os.sep, "_") if tag else "run"
        subtask7_5_diff_plot(
            t,
            pos_interp,
            truth_pos,
            vel_interp,
            truth_vel,
            run_id,
            out_dir,
        )
        print(
            f"Saved {Path(out_dir) / (run_id + '_task7_5_diff_truth_fused_over_time.pdf')}"
        )
    else:
        print("Subtask 7.5 skipped: missing fused or truth data")

    rmse_pos = float(np.sqrt(np.mean(norm_pos**2)))
    rmse_vel = float(np.sqrt(np.mean(norm_vel**2)))
    rmse_acc = float(np.sqrt(np.mean(norm_acc**2)))
    final_pos = float(norm_pos[-1])
    final_vel = float(norm_vel[-1])
    final_acc = float(norm_acc[-1])

    table = [
        ["Position [m]", final_pos, rmse_pos],
        ["Velocity [m/s]", final_vel, rmse_vel],
        ["Acceleration [m/s^2]", final_acc, rmse_acc],
    ]
    print(tabulate(table, headers=["Metric", "Final Error", "RMSE"], floatfmt=".3f"))

    runtime = time.time() - start_time
    method = tag.split("_")[-1] if tag else "unknown"
    print(
        f"[SUMMARY] method={method} rmse_pos={rmse_pos:.3f}m final_pos={final_pos:.3f}m "
        f"rmse_vel={rmse_vel:.3f}m/s final_vel={final_vel:.3f}m/s runtime={runtime:.2f}s"
    )


def subtask7_5_diff_plot(
    time: np.ndarray,
    fused_pos_ned: np.ndarray,
    truth_pos_ned: np.ndarray,
    fused_vel_ned: np.ndarray,
    truth_vel_ned: np.ndarray,
    run_id: str,
    out_dir: str,
) -> None:
    """Plot ``truth - fused`` position and velocity differences."""

    diff_pos_ned = truth_pos_ned - fused_pos_ned
    diff_vel_ned = truth_vel_ned - fused_vel_ned

    labels = ["North", "East", "Down"]
    fig, axes = plt.subplots(2, 3, figsize=(9, 4), sharex=True)
    for i in range(3):
        axes[0, i].plot(time, diff_pos_ned[:, i])
        axes[0, i].set_title(labels[i])
        axes[0, i].set_ylabel("Difference [m]")
        axes[0, i].grid(True)

        axes[1, i].plot(time, diff_vel_ned[:, i])
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].set_ylabel("Difference [m/s]")
        axes[1, i].grid(True)

    fig.suptitle("Truth - Fused Differences")
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf = out_dir / f"{run_id}_task7_5_diff_truth_fused_over_time.pdf"
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)

    pos_thr = 1.0
    vel_thr = 1.0
    for i, lab in enumerate(labels):
        dp = diff_pos_ned[:, i]
        dv = diff_vel_ned[:, i]
        print(
            f"{lab} position difference range: {dp.min():.2f} m to {dp.max():.2f} m.",
            end=" ",
        )
        idx_p = np.where(np.abs(dp) > pos_thr)[0]
        if idx_p.size:
            print(f"{idx_p.size} samples exceed {pos_thr:.1f} m")
        else:
            print(f"No samples exceed {pos_thr:.1f} m")

        print(
            f"{lab} velocity difference range: {dv.min():.2f} m/s to {dv.max():.2f} m/s.",
            end=" ",
        )
        idx_v = np.where(np.abs(dv) > vel_thr)[0]
        if idx_v.size:
            print(f"{idx_v.size} samples exceed {vel_thr:.1f} m/s")
        else:
            print(f"No samples exceed {vel_thr:.1f} m/s")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Evaluate filter results")
    ap.add_argument("--prediction")
    ap.add_argument("--gnss")
    ap.add_argument("--attitude")
    ap.add_argument("--npz", help="NPZ file produced by GNSS_IMU_Fusion.py")
    ap.add_argument("--output", default="plots/task7/")
    ap.add_argument("--tag", help="Dataset tag used as filename prefix")
    args = ap.parse_args()
    if args.npz:
        run_evaluation_npz(args.npz, args.output, args.tag)
    else:
        if not (args.prediction and args.gnss and args.attitude):
            ap.error(
                "--prediction, --gnss and --attitude are required when --npz is not given"
            )
        run_evaluation(args.prediction, args.gnss, args.attitude, args.output, args.tag)
