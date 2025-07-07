#!/usr/bin/env python3
"""Task 7 – Evaluation of Filter Results.

This module compares the predicted state from the Kalman filter
against the recorded GNSS measurements and visualises the attitude
history.  Residual statistics are printed and basic plots are saved
under ``plots/task7/``.
"""
from __future__ import annotations
from pathlib import Path
from typing import Sequence

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


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
) -> None:
    """Compute residuals and plot attitude angles."""
    out_dir = Path(save_path)
    out_dir.mkdir(parents=True, exist_ok=True)

    pred = pd.read_csv(prediction_file)
    gnss = pd.read_csv(gnss_file)
    att = pd.read_csv(attitude_file)

    for df in (pred, gnss, att):
        if "time" not in df.columns:
            for cand in ["t", "timestamp", "Time", "Time_s"]:
                if cand in df.columns:
                    df.rename(columns={cand: "time"}, inplace=True)
                    break

    pos_cols_p = _find_cols(pred, [["px", "py", "pz"], ["pos_x", "pos_y", "pos_z"], ["x", "y", "z"]])
    vel_cols_p = _find_cols(pred, [["vx", "vy", "vz"], ["vel_x", "vel_y", "vel_z"]])
    pos_cols_g = _find_cols(gnss, [["x", "y", "z"], ["pos_x", "pos_y", "pos_z"], ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]])
    vel_cols_g = _find_cols(gnss, [["vx", "vy", "vz"], ["vel_x", "vel_y", "vel_z"], ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]])
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
    fig.suptitle("GNSS - Predicted Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_dir / "residuals_position_velocity.pdf")
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
    fig.suptitle("Attitude Angles")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_dir / "attitude_angles_euler.pdf")
    plt.close(fig)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Evaluate filter results")
    ap.add_argument("--prediction", required=True)
    ap.add_argument("--gnss", required=True)
    ap.add_argument("--attitude", required=True)
    ap.add_argument("--output", default="plots/task7/")
    args = ap.parse_args()
    run_evaluation(args.prediction, args.gnss, args.attitude, args.output)
