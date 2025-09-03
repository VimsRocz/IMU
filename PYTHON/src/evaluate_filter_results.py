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
import json
import logging

from utils.plot_save import save_plot, task_summary

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from tabulate import tabulate
import time
from velocity_utils import derive_velocity
from utils import compute_C_ECEF_to_NED, ecef_to_geodetic

logger = logging.getLogger(__name__)


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
    logger.info(
        "run_evaluation inputs: prediction=%s gnss=%s attitude=%s out=%s tag=%s",
        prediction_file,
        gnss_file,
        attitude_file,
        save_path,
        tag,
    )
    out_dir = Path(save_path)
    # All Task 7 plots are written directly into ``results/``
    # unless a different directory is provided.
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
    metrics = {
        "mean_pos": mean_pos.tolist(),
        "std_pos": std_pos.tolist(),
        "mean_vel": mean_vel.tolist(),
        "std_vel": std_vel.tolist(),
    }
    metrics_json = out_dir / f"{tag or Path(prediction_file).stem}_residual_metrics.json"
    with metrics_json.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    np.savez(out_dir / f"{tag or Path(prediction_file).stem}_residual_metrics.npz", **metrics)
    logger.info("Saved residual metrics to %s", metrics_json)

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
    out_path = save_plot(fig, out_dir, tag or "run", "task7", "3_residuals_position_velocity", ext="pdf")
    try:
        from utils import save_plot_mat, save_plot_fig
        save_plot_mat(fig, str(out_path.with_suffix(".mat")))
        save_plot_fig(fig, str(out_path.with_suffix(".fig")))
    except Exception:
        pass
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
        hist_path = save_plot(fig, out_dir, tag or "run", "task7", f"hist_{name}_residuals", ext="pdf")
        try:
            from utils import save_plot_mat, save_plot_fig
            save_plot_mat(fig, str(hist_path.with_suffix(".mat")))
            save_plot_fig(fig, str(hist_path.with_suffix(".fig")))
        except Exception:
            pass
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
    att_out = save_plot(fig, out_dir, tag or "run", "task7", "4_attitude_angles_euler", ext="pdf")
    try:
        from utils import save_plot_mat, save_plot_fig
        save_plot_mat(fig, str(att_out.with_suffix(".mat")))
        save_plot_fig(fig, str(att_out.with_suffix(".fig")))
    except Exception:
        pass
    plt.close(fig)
    task_summary("task7")


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
    logger.info(
        "run_evaluation_npz inputs: npz=%s out=%s tag=%s",
        npz_file,
        save_path,
        tag,
    )
    start_time = time.time()
    out_dir = Path(save_path)
    out_dir.mkdir(parents=True, exist_ok=True)

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

    # Use relative time for all Task 7 plots so results align with Task 6
    t_rel = t - t[0]

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
    metrics = {
        "mean_pos": mean_pos,
        "std_pos": std_pos,
        "mean_vel": mean_vel,
        "std_vel": std_vel,
    }
    metrics_json = out_dir / f"{tag or Path(npz_file).stem}_residual_metrics.json"
    with metrics_json.open("w", encoding="utf-8") as f:
        json.dump({k: v.tolist() for k, v in metrics.items()}, f, indent=2)
    np.savez(out_dir / f"{tag or Path(npz_file).stem}_residual_metrics.npz", **metrics)
    logger.info("Saved residual metrics to %s", metrics_json)

    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
    for i in range(3):
        axes[0, i].plot(t_rel, res_pos[:, i])
        axes[0, i].set_title(labels[i])
        axes[0, i].set_ylabel("Pos Residual [m]")
        axes[0, i].grid(True)
        axes[1, i].plot(t_rel, res_vel[:, i])
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].set_ylabel("Vel Residual [m/s]")
        axes[1, i].grid(True)
    fig.suptitle("Task 7 – GNSS - Predicted Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = save_plot(fig, out_dir, tag or "run", "task7", "3_residuals_position_velocity", ext="pdf")
    plt.close(fig)

    rot = R.from_quat(quat[:, [1, 2, 3, 0]])
    euler = rot.as_euler("xyz", degrees=True)

    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    names = ["Roll", "Pitch", "Yaw"]
    for i in range(3):
        axs[i].plot(t_rel, euler[:, i])
        axs[i].set_ylabel(f"{names[i]} [deg]")
        axs[i].grid(True)
    axs[2].set_xlabel("Time [s]")
    fig.suptitle("Task 7 – Attitude Angles")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    att_path = save_plot(fig, out_dir, tag or "run", "task7", "4_attitude_angles_euler", ext="pdf")
    plt.close(fig)

    # Error norm plots
    norm_pos = np.linalg.norm(res_pos, axis=1)
    norm_vel = np.linalg.norm(res_vel, axis=1)
    res_acc = np.gradient(res_vel, t_rel, axis=0)
    norm_acc = np.linalg.norm(res_acc, axis=1)

    fig, ax = plt.subplots()
    ax.plot(t_rel, norm_pos, label="|pos error|")
    ax.plot(t_rel, norm_vel, label="|vel error|")
    ax.plot(t_rel, norm_acc, label="|acc error|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error Norm")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    norm_path = save_plot(fig, out_dir, tag or "run", "task7", "3_error_norms", ext="pdf")
    plt.close(fig)

    # Subtasks 7.5 and 7.6: difference and overlay (Truth vs Fused)
    if fused_time is not None and fused_pos is not None and fused_vel is not None:
        run_id = tag.replace(os.sep, "_") if tag else "run"

        ref_lat = data.get("ref_lat_rad")
        if ref_lat is None:
            v = data.get("ref_lat")
            if v is not None:
                ref_lat = float(np.asarray(v).squeeze())
        ref_lon = data.get("ref_lon_rad")
        if ref_lon is None:
            v = data.get("ref_lon")
            if v is not None:
                ref_lon = float(np.asarray(v).squeeze())
        if (ref_lat is None or ref_lon is None) and data.get("pos_ecef_m") is not None:
            lat_deg, lon_deg, _ = ecef_to_geodetic(*data["pos_ecef_m"][0])
            if ref_lat is None:
                ref_lat = np.deg2rad(lat_deg)
            if ref_lon is None:
                ref_lon = np.deg2rad(lon_deg)

        subtask7_5_diff_plot(
            t_rel,
            pos_interp,
            truth_pos,
            vel_interp,
            truth_vel,
            quat,
            ref_lat,
            ref_lon,
            run_id,
            out_dir,
        )
        # Task 7.6 overlays (Truth vs Fused) in all frames
        subtask7_6_overlay_plot(
            t_rel,
            pos_interp,
            truth_pos,
            vel_interp,
            truth_vel,
            quat,
            ref_lat,
            ref_lon,
            run_id,
            out_dir,
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
    logger.info(
        "Evaluation complete for %s: rmse_pos=%.3f final_pos=%.3f", npz_file, rmse_pos, final_pos
    )


def subtask7_5_diff_plot(
    time: np.ndarray,
    fused_pos_ned: np.ndarray,
    truth_pos_ned: np.ndarray,
    fused_vel_ned: np.ndarray,
    truth_vel_ned: np.ndarray,
    quat_bn: np.ndarray,
    ref_lat: float | None,
    ref_lon: float | None,
    run_id: str,
    out_dir: str,
) -> None:
    """Plot ``truth - fused`` differences in NED, ECEF and Body frames."""

    # Use relative time for direct comparison with Task 6 plots
    time = time - time[0]

    diff_pos_ned = truth_pos_ned - fused_pos_ned
    diff_vel_ned = truth_vel_ned - fused_vel_ned

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    def _compute_frame_summary(arr_p: np.ndarray, arr_v: np.ndarray, time_vec: np.ndarray):
        """Return dict with Final Error and RMSE for position/velocity/acceleration."""
        # Vector norms
        pos_norm = np.linalg.norm(arr_p, axis=1)
        vel_norm = np.linalg.norm(arr_v, axis=1)
        # Differentiate velocity to get acceleration difference
        try:
            acc = np.gradient(arr_v, time_vec, axis=0)
        except Exception:
            # Fallback: uniform dt
            dt = np.median(np.diff(time_vec)) if len(time_vec) > 1 else 1.0
            acc = np.diff(arr_v, axis=0, prepend=arr_v[:1]) / max(dt, 1e-12)
        acc_norm = np.linalg.norm(acc, axis=1)

        def _final(arr):
            return float(arr[-1]) if arr.size else float("nan")

        def _rmse(arr):
            return float(np.sqrt(np.mean(arr ** 2))) if arr.size else float("nan")

        return {
            "Position [m]": {"Final Error": _final(pos_norm), "RMSE": _rmse(pos_norm)},
            "Velocity [m/s]": {"Final Error": _final(vel_norm), "RMSE": _rmse(vel_norm)},
            "Acceleration [m/s^2]": {"Final Error": _final(acc_norm), "RMSE": _rmse(acc_norm)},
        }

    def _print_and_save_summary(summary: dict, frame: str, out_dir: Path, run_id: str):
        headers = ["Metric", "Final Error", "RMSE"]
        rows = [
            [k, v.get("Final Error", float("nan")), v.get("RMSE", float("nan"))]
            for k, v in summary.items()
        ]
        print(f"\n{frame} frame differences summary:")
        print(tabulate(rows, headers=headers, floatfmt=".3f"))
        # save CSV alongside plots
        df = pd.DataFrame(rows, columns=headers)
        csv_path = out_dir / f"{run_id}_task7_5_{frame.lower()}_summary.csv"
        try:
            df.to_csv(csv_path, index=False)
        except Exception:
            pass

    def _plot(arr_p: np.ndarray, arr_v: np.ndarray, labels: list[str], frame: str) -> None:
        # Decimate to keep plots responsive for very long runs
        n = len(time)
        stride = int(np.ceil(n / 200000)) if n > 200000 else 1
        t_plot = time[::stride]
        p_plot = arr_p[::stride]
        v_plot = arr_v[::stride]

        fig, axes = plt.subplots(2, 3, figsize=(9, 4), sharex=True)
        for i in range(3):
            axes[0, i].plot(t_plot, p_plot[:, i])
            axes[0, i].set_title(labels[i])
            axes[0, i].set_ylabel("Difference [m]")
            axes[0, i].grid(True)

            axes[1, i].plot(t_plot, v_plot[:, i])
            axes[1, i].set_xlabel("Time [s]")
            axes[1, i].set_ylabel("Difference [m/s]")
            axes[1, i].grid(True)

        # Harmonise y-limits per row using robust symmetric limits (99.5th percentile)
        try:
            lim_p = float(np.percentile(np.abs(p_plot), 99.5))
            lim_v = float(np.percentile(np.abs(v_plot), 99.5))
            lim_p = lim_p if np.isfinite(lim_p) and lim_p > 0 else None
            lim_v = lim_v if np.isfinite(lim_v) and lim_v > 0 else None
            if lim_p:
                for i in range(3):
                    axes[0, i].set_ylim(-lim_p, lim_p)
            if lim_v:
                for i in range(3):
                    axes[1, i].set_ylim(-lim_v, lim_v)
        except Exception:
            pass

        fig.suptitle(f"Truth - Fused Differences ({frame} Frame)")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        base_label = f"5_diff_truth_fused_over_time_{frame}"
        pdf = save_plot(fig, out_dir, run_id, "task7", base_label, ext="pdf")
        png = save_plot(fig, out_dir, run_id, "task7", base_label, ext="png")
        try:
            from utils import save_plot_mat, save_plot_fig
            save_plot_mat(fig, str(pdf.with_suffix(".mat")))
            save_plot_fig(fig, str(pdf.with_suffix(".fig")))
        except Exception:
            pass
        plt.close(fig)

        pos_thr = 1.0
        vel_thr = 1.0
        for i, lab in enumerate(labels):
            dp = arr_p[:, i]
            dv = arr_v[:, i]
            print(
                f"{frame} {lab} position diff range: {dp.min():.2f} m to {dp.max():.2f} m.",
                end=" ",
            )
            idx_p = np.where(np.abs(dp) > pos_thr)[0]
            if idx_p.size:
                print(f"{idx_p.size} samples exceed {pos_thr:.1f} m")
            else:
                print(f"No samples exceed {pos_thr:.1f} m")

            print(
                f"{frame} {lab} velocity diff range: {dv.min():.2f} m/s to {dv.max():.2f} m/s.",
                end=" ",
            )
            idx_v = np.where(np.abs(dv) > vel_thr)[0]
            if idx_v.size:
                print(f"{idx_v.size} samples exceed {vel_thr:.1f} m/s")
            else:
                print(f"No samples exceed {vel_thr:.1f} m/s")

    # NED frame
    _plot(diff_pos_ned, diff_vel_ned, ["North", "East", "Down"], "NED")
    _print_and_save_summary(_compute_frame_summary(diff_pos_ned, diff_vel_ned, time), "NED", out_dir, run_id)

    # ECEF frame
    if ref_lat is not None and ref_lon is not None:
        C = compute_C_ECEF_to_NED(ref_lat, ref_lon).T
    else:
        C = np.eye(3)
    diff_pos_ecef = (C @ diff_pos_ned.T).T
    diff_vel_ecef = (C @ diff_vel_ned.T).T
    _plot(diff_pos_ecef, diff_vel_ecef, ["X", "Y", "Z"], "ECEF")
    _print_and_save_summary(_compute_frame_summary(diff_pos_ecef, diff_vel_ecef, time), "ECEF", out_dir, run_id)

    # Body frame
    rot = R.from_quat(quat_bn[:, [1, 2, 3, 0]])
    diff_pos_body = rot.apply(diff_pos_ned, inverse=True)
    diff_vel_body = rot.apply(diff_vel_ned, inverse=True)
    _plot(diff_pos_body, diff_vel_body, ["X", "Y", "Z"], "Body")
    _print_and_save_summary(_compute_frame_summary(diff_pos_body, diff_vel_body, time), "Body", out_dir, run_id)
    task_summary("task7")
    print(
        "Saved Task 7.5 diff-truth plots for NED/ECEF/Body frames under: "
        f"{out_dir}/"
    )

    # Persist differences for downstream analysis
    try:
        np.savez(
            out_dir / f"{run_id}_task7_5_diffs.npz",
            time=time,
            pos_ned=diff_pos_ned,
            vel_ned=diff_vel_ned,
            pos_ecef=diff_pos_ecef,
            vel_ecef=diff_vel_ecef,
            pos_body=diff_pos_body,
            vel_body=diff_vel_body,
        )
    except Exception:
        pass


def subtask7_6_overlay_plot(
    time: np.ndarray,
    fused_pos_ned: np.ndarray,
    truth_pos_ned: np.ndarray,
    fused_vel_ned: np.ndarray,
    truth_vel_ned: np.ndarray,
    quat_bn: np.ndarray,
    ref_lat: float | None,
    ref_lon: float | None,
    run_id: str,
    out_dir: str,
) -> None:
    """Plot Truth and Fused overlays in NED, ECEF and Body frames (Task 7.6)."""

    time = time - time[0]
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    def _plot_overlay(arr_p_est, arr_p_tru, arr_v_est, arr_v_tru, labels, frame):
        # Decimate if needed to keep plots responsive
        n = len(time)
        stride = int(np.ceil(n / 200000)) if n > 200000 else 1
        t_plot = time[::stride]
        p_est = arr_p_est[::stride]
        p_tru = arr_p_tru[::stride]
        v_est = arr_v_est[::stride]
        v_tru = arr_v_tru[::stride]

        fig, axes = plt.subplots(2, 3, figsize=(9, 4), sharex=True)
        for i in range(3):
            axes[0, i].plot(t_plot, p_est[:, i], label="Fused")
            axes[0, i].plot(t_plot, p_tru[:, i], '--', label="Truth")
            axes[0, i].set_title(labels[i])
            axes[0, i].set_ylabel("Position [m]")
            axes[0, i].grid(True)
            axes[1, i].plot(t_plot, v_est[:, i], label="Fused")
            axes[1, i].plot(t_plot, v_tru[:, i], '--', label="Truth")
            axes[1, i].set_xlabel("Time [s]")
            axes[1, i].set_ylabel("Velocity [m/s]")
            axes[1, i].grid(True)

        # Harmonise y-limits using robust symmetric limits across columns
        try:
            lim_p = float(np.percentile(np.abs(np.concatenate([p_est, p_tru], axis=0)), 99.5))
            lim_v = float(np.percentile(np.abs(np.concatenate([v_est, v_tru], axis=0)), 99.5))
            if lim_p and np.isfinite(lim_p) and lim_p > 0:
                for i in range(3):
                    axes[0, i].set_ylim(-lim_p, lim_p)
            if lim_v and np.isfinite(lim_v) and lim_v > 0:
                for i in range(3):
                    axes[1, i].set_ylim(-lim_v, lim_v)
        except Exception:
            pass
        handles, labels_h = axes[0, 0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labels_h, ncol=2, loc="upper center")
        fig.suptitle(f"Task 7.6 – Truth vs Fused ({frame} Frame)")
        fig.tight_layout(rect=[0, 0, 1, 0.92])
        base = out_dir / f"{run_id}_task7_6_overlay_{frame}"
        try:
            fig.savefig(base.with_suffix('.png'), dpi=200, bbox_inches='tight')
            fig.savefig(base.with_suffix('.pdf'), dpi=200, bbox_inches='tight')
        except Exception:
            pass
        plt.close(fig)

    # NED overlay
    _plot_overlay(
        fused_pos_ned, truth_pos_ned, fused_vel_ned, truth_vel_ned, ["North", "East", "Down"], "NED"
    )

    # ECEF overlay (static rotation using reference lat/lon)
    if ref_lat is not None and ref_lon is not None:
        C = compute_C_ECEF_to_NED(ref_lat, ref_lon).T
    else:
        C = np.eye(3)
    fused_pos_ecef = (C @ fused_pos_ned.T).T
    truth_pos_ecef = (C @ truth_pos_ned.T).T
    fused_vel_ecef = (C @ fused_vel_ned.T).T
    truth_vel_ecef = (C @ truth_vel_ned.T).T
    _plot_overlay(
        fused_pos_ecef, truth_pos_ecef, fused_vel_ecef, truth_vel_ecef, ["X", "Y", "Z"], "ECEF"
    )

    # Body overlay (rotate NED by quaternion body->NED)
    rot = R.from_quat(quat_bn[:, [1, 2, 3, 0]])
    fused_pos_body = rot.apply(fused_pos_ned, inverse=True)
    truth_pos_body = rot.apply(truth_pos_ned, inverse=True)
    fused_vel_body = rot.apply(fused_vel_ned, inverse=True)
    truth_vel_body = rot.apply(truth_vel_ned, inverse=True)
    _plot_overlay(
        fused_pos_body, truth_pos_body, fused_vel_body, truth_vel_body, ["X", "Y", "Z"], "Body"
    )
    print("Saved Task 7.6 truth-vs-fused overlays (NED/ECEF/Body) under:", out_dir)

    # Persist overlay series for downstream analysis
    try:
        np.savez(
            out_dir / f"{run_id}_task7_6_overlay_data.npz",
            time=time,
            fused_pos_ned=fused_pos_ned,
            truth_pos_ned=truth_pos_ned,
            fused_vel_ned=fused_vel_ned,
            truth_vel_ned=truth_vel_ned,
            fused_pos_ecef=fused_pos_ecef,
            truth_pos_ecef=truth_pos_ecef,
            fused_vel_ecef=fused_vel_ecef,
            truth_vel_ecef=truth_vel_ecef,
            fused_pos_body=fused_pos_body,
            truth_pos_body=truth_pos_body,
            fused_vel_body=fused_vel_body,
            truth_vel_body=truth_vel_body,
        )
    except Exception:
        pass


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Evaluate filter results")
    ap.add_argument("--prediction")
    ap.add_argument("--gnss")
    ap.add_argument("--attitude")
    ap.add_argument("--npz", help="NPZ file produced by GNSS_IMU_Fusion.py")
    # Task 7 plots now default to the top-level ``results`` directory
    ap.add_argument("--output", default="results")
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
