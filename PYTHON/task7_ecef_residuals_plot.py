"""Task 7 – Plot position and velocity residuals in the ECEF frame.

Usage:
    python task7_ecef_residuals_plot.py --est-file <fused.npz> \
        --imu-file <IMU.dat> --gnss-file <GNSS.csv> [--truth-file <STATE_X.txt>] \
        --output-dir results

This script loads a fused estimator output file and the corresponding
ground truth trajectory.  If the estimator output already contains the
``truth_pos_ecef``, ``truth_vel_ecef`` and ``truth_time`` fields produced
by Task 4, the ``--truth-file`` argument is optional.  Otherwise the
STATE_X text log must be provided or inferable from the dataset name.
The truth trajectory is synchronised to the estimator by cross-correlating
position and velocity magnitudes before interpolation to the estimator time
vector.  Position, velocity and acceleration residuals are plotted.  The time axis is
converted to ``t - t[0]`` so Task 6 and Task 7 share the same reference.
Figures are saved as PDF and PNG under ``results/<tag>/`` where ``tag``
combines the dataset, GNSS file and method.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from validate_with_truth import load_estimate, assemble_frames
from naming import make_tag, plot_filename
import re


# Verbose logging toggle set in main()
VERBOSE = False


def log_step(msg: str) -> None:
    print(f"[Task7] {msg}")


def vlog(msg: str) -> None:
    if VERBOSE:
        print(f"[Task7] {msg}")


def compute_residuals(
    t_est: np.ndarray,
    est_pos: np.ndarray,
    est_vel: np.ndarray,
    truth_pos: np.ndarray,
    truth_vel: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return position, velocity and acceleration residuals."""
    res_pos = est_pos - truth_pos
    res_vel = est_vel - truth_vel
    acc_est = np.gradient(est_vel, t_est, axis=0)
    acc_truth = np.gradient(truth_vel, t_est, axis=0)
    res_acc = acc_est - acc_truth
    return res_pos, res_vel, res_acc


def plot_residuals(
    t: np.ndarray,
    res_pos: np.ndarray,
    res_vel: np.ndarray,
    res_acc: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> None:
    """Plot residual components and norms."""
    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)

    for i, (arr, ylabel) in enumerate(
        [
            (res_pos, "Position Residual [m]"),
            (res_vel, "Velocity Residual [m/s]"),
            (res_acc, "Acceleration Residual [m/s$^2$]"),
        ]
    ):
        for j in range(3):
            ax = axes[i, j]
            ax.plot(t, arr[:, j])
            if i == 0:
                ax.set_title(labels[j])
            if j == 0:
                ax.set_ylabel(ylabel)
            if i == 2:
                ax.set_xlabel("Time [s]")
            ax.grid(True)

    tag = make_tag(dataset, gnss, method)
    fig.suptitle(f"{tag} Task 7 ECEF Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_name = plot_filename(dataset, gnss, method, 7, "3", "ecef_residuals")
    pdf = out_dir / pdf_name
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)

    # Norm plot
    fig, ax = plt.subplots()
    ax.plot(t, np.linalg.norm(res_pos, axis=1), label="|pos|")
    ax.plot(t, np.linalg.norm(res_vel, axis=1), label="|vel|")
    ax.plot(t, np.linalg.norm(res_acc, axis=1), label="|acc|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Residual Norm")
    ax.legend()
    ax.grid(True)
    fig.suptitle(f"{tag} Task 7 ECEF Residual Norms")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    norm_name = plot_filename(dataset, gnss, method, 7, "3", "ecef_residual_norms")
    norm_pdf = out_dir / norm_name
    norm_png = norm_pdf.with_suffix(".png")
    fig.savefig(norm_pdf)
    fig.savefig(norm_png)
    plt.close(fig)

    saved = sorted(out_dir.glob(f"{tag}_task7_*ecef_residual*.pdf"))
    if saved:
        print("Files saved in", out_dir)
        for f in saved:
            print(" -", f.name)


def compute_attitude_truth_vs_est(
    est: dict,
    truth_file: str,
    t_est_ref: np.ndarray,
    pos_est_ecef: np.ndarray,
    vel_est_ecef: np.ndarray,
) -> dict | None:
    """Compute aligned truth vs estimate attitude data and angle error.

    Returns a dict with keys:
      - t_rel: time vector [s] relative to first sample
      - q_truth_wxyz, q_est_wxyz: quaternions aligned to t_rel in wxyz
      - eul_truth_zyx_deg, eul_est_zyx_deg: Euler (ZYX) in degrees
      - angle_deg: quaternion angle error [deg]

    Returns ``None`` when not available.
    """
    try:
        from scipy.spatial.transform import Rotation as R, Slerp
    except Exception:
        # SciPy not available in some environments
        return None

    # Require estimate quaternions and a truth file with quaternions
    q_est_raw = est.get("quat")
    if q_est_raw is None or truth_file is None:
        return None
    try:
        truth = np.loadtxt(truth_file, comments="#")
    except Exception:
        return None

    if truth.shape[1] < 12:
        # No quaternion columns in truth
        return None

    # Times
    t_est_all = np.asarray(est.get("time"))
    if t_est_all is None or t_est_all.size == 0:
        t_est_all = np.arange(len(q_est_raw)) * 0.0025
    else:
        t_est_all = np.asarray(t_est_all).squeeze()

    # Cross-correlation-based time offset between estimate (ECEF) and truth (ECEF)
    t_truth = truth[:, 1]
    pos_truth = truth[:, 2:5]
    vel_truth = truth[:, 5:8]

    # Build a common grid for correlation using estimate reference timebase
    # Use t_est_ref from frames (already zero-based) for robust rate estimate
    dt_r = max(np.mean(np.diff(t_est_ref)), np.mean(np.diff(t_truth)))
    if not np.isfinite(dt_r) or dt_r <= 0:
        dt_r = (max(t_est_ref[-1], t_truth[-1]) - min(t_est_ref[0], t_truth[0])) / max(
            len(t_est_ref), len(t_truth), 1
        )
    t_start = min(t_est_ref[0], t_truth[0])
    t_end = max(t_est_ref[-1], t_truth[-1])
    t_grid = np.arange(t_start, t_end + dt_r, dt_r)

    def interp_to(t_src, arr):
        return np.vstack([np.interp(t_grid, t_src, arr[:, i]) for i in range(arr.shape[1])]).T

    pos_est_rs = interp_to(t_est_ref, pos_est_ecef)
    vel_est_rs = interp_to(t_est_ref, vel_est_ecef)
    pos_truth_rs = interp_to(t_truth, pos_truth)
    vel_truth_rs = interp_to(t_truth, vel_truth)

    # Cross-correlate magnitudes to estimate time offset
    def best_lag(a, b):
        a = a - a.mean()
        b = b - b.mean()
        lags = np.arange(-len(b) + 1, len(a))
        idx = np.argmax(np.correlate(a, b, mode="full"))
        return lags[idx]

    lag_pos = best_lag(np.linalg.norm(pos_est_rs, axis=1), np.linalg.norm(pos_truth_rs, axis=1))
    lag_vel = best_lag(np.linalg.norm(vel_est_rs, axis=1), np.linalg.norm(vel_truth_rs, axis=1))
    time_offset = 0.5 * (lag_pos + lag_vel) * dt_r
    time_offset = float(np.clip(time_offset, -5.0, 5.0))

    # Prepare Slerp for truth and estimate
    q_truth_wxyz = truth[:, 8:12]
    # SciPy expects xyzw
    r_truth = R.from_quat(q_truth_wxyz[:, [1, 2, 3, 0]])
    s_truth = Slerp(t_truth + time_offset, r_truth)

    q_est_wxyz = np.asarray(q_est_raw)
    # Ensure equal lengths for Slerp
    n_est = min(len(t_est_all), len(q_est_wxyz))
    if n_est <= 1:
        return None
    t_est_all = t_est_all[:n_est]
    q_est_wxyz = q_est_wxyz[:n_est]
    r_est = R.from_quat(q_est_wxyz[:, [1, 2, 3, 0]])
    s_est = Slerp(t_est_all, r_est)

    # Sample both at estimator time within overlap window
    t0 = max(float((t_truth + time_offset)[0]), float(t_est_all[0]))
    t1 = min(float((t_truth + time_offset)[-1]), float(t_est_all[-1]))
    if not (t1 > t0):
        return None
    mask = (t_est_all >= t0) & (t_est_all <= t1)
    t_eval = t_est_all[mask]
    if t_eval.size == 0:
        return None

    r_t = s_truth(np.clip(t_eval, (t_truth + time_offset)[0], (t_truth + time_offset)[-1]))
    r_e = s_est(np.clip(t_eval, t_est_all[0], t_est_all[-1]))

    # Quaternion angle error (shortest arc), in degrees
    q_t_xyzw = r_t.as_quat()
    q_e_xyzw = r_e.as_quat()
    # Convert to wxyz for dot product convenience
    q_t = np.column_stack([q_t_xyzw[:, 3], q_t_xyzw[:, 0], q_t_xyzw[:, 1], q_t_xyzw[:, 2]])
    q_e = np.column_stack([q_e_xyzw[:, 3], q_e_xyzw[:, 0], q_e_xyzw[:, 1], q_e_xyzw[:, 2]])
    q_t = q_t / np.linalg.norm(q_t, axis=1, keepdims=True)
    q_e = q_e / np.linalg.norm(q_e, axis=1, keepdims=True)
    dot = np.clip(np.abs(np.sum(q_t * q_e, axis=1)), 0.0, 1.0)
    ang_deg = 2.0 * np.degrees(np.arccos(dot))

    # Euler ZYX (yaw,pitch,roll) for both
    eul_truth_zyx_deg = r_t.as_euler("zyx", degrees=True)
    eul_est_zyx_deg = r_e.as_euler("zyx", degrees=True)

    # Return time relative to start for nicer axis
    t_rel = t_eval - t_eval[0]
    return {
        "t_rel": t_rel,
        "q_truth_wxyz": q_t,
        "q_est_wxyz": q_e,
        "eul_truth_zyx_deg": eul_truth_zyx_deg,
        "eul_est_zyx_deg": eul_est_zyx_deg,
        "angle_deg": ang_deg,
        "time_offset": time_offset,
    }


def compute_attitude_angle_error(
    est: dict,
    truth_file: str,
    t_est_ref: np.ndarray,
    pos_est_ecef: np.ndarray,
    vel_est_ecef: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    """Backwards-compatible wrapper to return only time and angle error."""
    d = compute_attitude_truth_vs_est(
        est, truth_file, t_est_ref, pos_est_ecef, vel_est_ecef
    )
    if d is None:
        return None
    return d["t_rel"], d["angle_deg"]


def plot_quaternion_truth_vs_est(
    t_rel: np.ndarray,
    q_truth_wxyz: np.ndarray,
    q_est_wxyz: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Plot quaternion components (w,x,y,z) truth vs estimate for Task 7.6."""
    tag = make_tag(dataset, gnss, method)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    labels = ["w", "x", "y", "z"]
    for i, lab in enumerate(labels):
        axes[i].plot(t_rel, q_truth_wxyz[:, i], label="truth", alpha=0.9)
        axes[i].plot(t_rel, q_est_wxyz[:, i], label="estimate", alpha=0.9)
        axes[i].set_ylabel(f"q_{lab}")
        axes[i].grid(True)
        if i == 0:
            axes[i].legend(loc="upper right")
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{tag} Task 7.6 Attitude Quaternions: Truth vs Estimate")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_quaternion_truth_vs_estimate")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def plot_euler_truth_vs_est(
    t_rel: np.ndarray,
    eul_truth_zyx_deg: np.ndarray,
    eul_est_zyx_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Plot Euler ZYX (yaw, pitch, roll) truth vs estimate for Task 7.6."""
    tag = make_tag(dataset, gnss, method)
    fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    labels = ["Yaw [deg]", "Pitch [deg]", "Roll [deg]"]
    for i, lab in enumerate(labels):
        axes[i].plot(t_rel, eul_truth_zyx_deg[:, i], label="truth", alpha=0.9)
        axes[i].plot(t_rel, eul_est_zyx_deg[:, i], label="estimate", alpha=0.9)
        axes[i].set_ylabel(lab)
        axes[i].grid(True)
        if i == 0:
            axes[i].legend(loc="upper right")
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{tag} Task 7.6 Attitude (Euler ZYX): Truth vs Estimate")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_euler_truth_vs_estimate")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def plot_quaternion_component_error_task76(
    t_rel: np.ndarray,
    q_truth_wxyz: np.ndarray,
    q_est_wxyz: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Plot per-component quaternion error (estimate - truth) for Task 7.6.

    Estimate quaternions are sign-aligned to truth per sample to handle the
    quaternion double-cover ambiguity.
    """
    dots = np.sum(q_truth_wxyz * q_est_wxyz, axis=1)
    signs = np.sign(dots)
    signs[signs == 0] = 1.0
    q_est_aligned = q_est_wxyz * signs[:, None]
    dq = q_est_aligned - q_truth_wxyz

    tag = make_tag(dataset, gnss, method)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    labels = ["w", "x", "y", "z"]
    for i, lab in enumerate(labels):
        axes[i].plot(t_rel, dq[:, i])
        axes[i].set_ylabel(f"Δq_{lab}")
        axes[i].grid(True)
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{tag} Task 7.6 Quaternion Component Error (est - truth)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_quaternion_error_components")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def plot_attitude_angle_error(
    t_rel: np.ndarray,
    ang_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Plot and save the quaternion angle error curve with standard naming."""
    tag = make_tag(dataset, gnss, method)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_rel, ang_deg, label="attitude error [deg]")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle error [deg]")
    ax.grid(True)
    fig.suptitle(f"{tag} Task 7 Attitude Angle Error")
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    out_dir.mkdir(parents=True, exist_ok=True)
    fname = plot_filename(dataset, gnss, method, 7, "4", "attitude_angle_error")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def plot_attitude_angle_error_task76(
    t_rel: np.ndarray,
    ang_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Same as plot_attitude_angle_error but saved under Task 7.6 naming."""
    tag = make_tag(dataset, gnss, method)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_rel, ang_deg, label="attitude error [deg]")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle error [deg]")
    ax.grid(True)
    fig.suptitle(f"{tag} Task 7.6 Attitude Angle Error")
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    out_dir.mkdir(parents=True, exist_ok=True)
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_angle_error")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def plot_euler_error_task76(
    t_rel: np.ndarray,
    e_err_zyx_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Plot Euler residuals (ZYX: yaw, pitch, roll) vs time under Task 7.6 naming."""
    tag = make_tag(dataset, gnss, method)
    fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    labels = ["Yaw [deg]", "Pitch [deg]", "Roll [deg]"]
    for i, lab in enumerate(labels):
        axes[i].plot(t_rel, e_err_zyx_deg[:, i])
        axes[i].set_ylabel(lab)
        axes[i].grid(True)
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{tag} Task 7.6 Euler Error (ZYX) vs Time")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_euler_error_over_time")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def _wrap_deg(x: np.ndarray) -> np.ndarray:
    x = (x + 180.0) % 360.0 - 180.0
    x[x == -180.0] = 180.0
    return x


def _quat_sign_align(q_ref_wxyz: np.ndarray, q_cmp_wxyz: np.ndarray) -> np.ndarray:
    dots = np.sum(q_ref_wxyz * q_cmp_wxyz, axis=1)
    signs = np.where(dots >= 0.0, 1.0, -1.0)
    return q_cmp_wxyz * signs[:, None]


def _quat_error_wxyz(q_truth_wxyz: np.ndarray, q_est_wxyz: np.ndarray) -> np.ndarray:
    """Quaternion error q_err = q_est * conj(q_truth), both wxyz; returns wxyz."""
    w1, x1, y1, z1 = q_est_wxyz.T
    w2, x2, y2, z2 = q_truth_wxyz.T
    # conj(truth)
    w2c, x2c, y2c, z2c = w2, -x2, -y2, -z2
    w = w1 * w2c - x1 * x2c - y1 * y2c - z1 * z2c
    x = w1 * x2c + x1 * w2c + y1 * z2c - z1 * y2c
    y = w1 * y2c - x1 * z2c + y1 * w2c + z1 * x2c
    z = w1 * z2c + x1 * y2c - y1 * x2c + z1 * w2c
    q = np.column_stack([w, x, y, z])
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def _angle_axis_error_vec_deg(q_err_wxyz: np.ndarray) -> np.ndarray:
    """Return angle-axis error vector (deg) from error quaternion (wxyz).

    For small errors, components approximate roll/pitch/yaw error in body axes.
    """
    w = np.clip(q_err_wxyz[:, 0], -1.0, 1.0)
    theta = 2.0 * np.arccos(np.abs(w))  # [rad]
    s = np.sqrt(1.0 - np.minimum(w * w, 1.0))
    # Avoid division by zero; when s ~ 0, axis is ill-defined -> set to zeros
    axis = np.zeros_like(q_err_wxyz[:, 1:])
    nz = s > 1e-9
    axis[nz] = q_err_wxyz[nz, 1:] / s[nz, None]
    err_vec_rad = (theta[:, None]) * axis
    return np.degrees(err_vec_rad)


def _stats(name: str, arr: np.ndarray) -> dict:
    arr = np.asarray(arr)
    if arr.ndim == 1:
        vals = arr
    else:
        vals = arr.ravel()
    d = {
        "mean": float(np.mean(vals)),
        "std": float(np.std(vals)),
        "rmse": float(np.sqrt(np.mean(vals**2))),
        "max_abs": float(np.max(np.abs(vals))) if vals.size else float("nan"),
    }
    return d


def _stats_by_col(arr: np.ndarray, labels: list[str]) -> dict[str, dict]:
    stats: dict[str, dict] = {}
    for i, lab in enumerate(labels):
        col = arr[:, i]
        stats[lab] = {
            "mean": float(np.mean(col)),
            "std": float(np.std(col)),
            "rmse": float(np.sqrt(np.mean(col**2))),
            "max_abs": float(np.max(np.abs(col))) if col.size else float("nan"),
        }
    return stats


def _print_table(title: str, labels: list[str], stats: dict[str, dict]) -> None:
    print(f"\n{title}")
    header = ["axis", "mean", "std", "rmse", "max|.|"]
    print(" ".join(f"{h:>10s}" for h in header))
    for lab in labels:
        s = stats[lab]
        row = [lab, f"{s['mean']:.3f}", f"{s['std']:.3f}", f"{s['rmse']:.3f}", f"{s['max_abs']:.3f}"]
        print(" ".join(f"{c:>10s}" for c in row))


def save_euler_error_summary_csv(
    e_err_zyx_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    labels = ["yaw", "pitch", "roll"]
    stats = _stats_by_col(e_err_zyx_deg, ["Yaw", "Pitch", "Roll"])
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_euler_error_summary", ext="csv")
    path = out_dir / fname
    out_dir.mkdir(parents=True, exist_ok=True)
    import csv
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["axis", "mean_deg", "std_deg", "rmse_deg", "max_abs_deg"])
        for lab_key, axis in zip(["Yaw", "Pitch", "Roll"], labels):
            s = stats[lab_key]
            w.writerow([axis, f"{s['mean']:.6f}", f"{s['std']:.6f}", f"{s['rmse']:.6f}", f"{s['max_abs']:.6f}"])
    return path


def save_quat_component_error_summary_csv(
    dq_wxyz: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    labels = ["q_w", "q_x", "q_y", "q_z"]
    stats = _stats_by_col(dq_wxyz, labels)
    fname = plot_filename(dataset, gnss, method, 7, "6", "attitude_quaternion_error_components_summary", ext="csv")
    path = out_dir / fname
    out_dir.mkdir(parents=True, exist_ok=True)
    import csv
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["component", "mean", "std", "rmse", "max_abs"])
        for axis in labels:
            s = stats[axis]
            w.writerow([axis, f"{s['mean']:.6f}", f"{s['std']:.6f}", f"{s['rmse']:.6f}", f"{s['max_abs']:.6f}"])
    return path


def save_angle_error_summary(
    ang_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
) -> Path:
    """Save a CSV table summarising quaternion angle error statistics."""
    import csv

    mean = float(np.mean(ang_deg))
    rmse = float(np.sqrt(np.mean(ang_deg**2)))
    p95 = float(np.percentile(ang_deg, 95))
    p99 = float(np.percentile(ang_deg, 99))
    maxv = float(np.max(ang_deg))
    final = float(ang_deg[-1])

    fname = plot_filename(dataset, gnss, method, 7, "4", "attitude_angle_error_summary", ext="csv")
    path = out_dir / fname
    out_dir.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value_deg"]) 
        for k, v in [
            ("mean", mean),
            ("rmse", rmse),
            ("p95", p95),
            ("p99", p99),
            ("max", maxv),
            ("final", final),
        ]:
            w.writerow([k, f"{v:.6f}"])
    return path


def plot_angle_error_summary(
    ang_deg: np.ndarray,
    dataset: str,
    gnss: str,
    method: str,
    out_dir: Path,
    subtask: str = "6",
) -> Path:
    """Plot a summary (bars + table) of quaternion angle error metrics.

    Saved under Task 7.<subtask> naming.
    """
    metrics = {
        "mean": float(np.mean(ang_deg)),
        "rmse": float(np.sqrt(np.mean(ang_deg ** 2))),
        "p95": float(np.percentile(ang_deg, 95)),
        "p99": float(np.percentile(ang_deg, 99)),
        "max": float(np.max(ang_deg)),
        "final": float(ang_deg[-1]),
    }

    labels = list(metrics.keys())
    values = [metrics[k] for k in labels]

    tag = make_tag(dataset, gnss, method)
    fig, ax = plt.subplots(figsize=(9, 4.5))
    bars = ax.bar(labels, values, color="#4C78A8")
    ax.set_ylabel("Angle [deg]")
    ax.set_title(f"{tag} Task 7.{subtask} Attitude Angle Error Summary")
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    for b in bars:
        ax.annotate(f"{b.get_height():.2f}", xy=(b.get_x() + b.get_width()/2, b.get_height()),
                    xytext=(0, 3), textcoords="offset points", ha="center", va="bottom", fontsize=8)

    # Add a table below the bars
    table_data = [[k, f"{metrics[k]:.6f}"] for k in labels]
    table = plt.table(cellText=table_data, colLabels=["metric", "value_deg"],
                      loc="bottom", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1, 1.2)
    plt.subplots_adjust(left=0.08, right=0.98, top=0.85, bottom=0.25)

    out_dir.mkdir(parents=True, exist_ok=True)
    fname = plot_filename(dataset, gnss, method, 7, subtask, "attitude_angle_error_summary_plot")
    pdf = out_dir / fname
    png = pdf.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png)
    plt.close(fig)
    return pdf


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot ECEF residuals for Task 7")
    ap.add_argument("--est-file", required=True, help="fused estimator .npz")
    ap.add_argument(
        "--imu-file", required=True, help="raw IMU file used for the estimate"
    )
    ap.add_argument(
        "--gnss-file", required=True, help="raw GNSS file used for the estimate"
    )
    ap.add_argument(
        "--truth-file",
        help=(
            "ground truth STATE_X file. If omitted the script uses truth data "
            "embedded in the estimator output or attempts to infer the path "
            "from --dataset"
        ),
    )
    ap.add_argument("--dataset", required=True, help="IMU dataset file")
    ap.add_argument("--gnss", required=True, help="GNSS dataset file")
    ap.add_argument("--method", required=True, help="initialisation method")
    ap.add_argument("--output-dir", default="results", help="directory for plots")
    ap.add_argument("-v", "--verbose", action="store_true", help="print step-by-step procedure")
    args = ap.parse_args()

    global VERBOSE
    VERBOSE = bool(args.verbose)

    log_step("Loading estimate file ...")
    est = load_estimate(args.est_file)
    vlog(f"Estimate keys: {list(est.keys())}")
    truth_file = args.truth_file
    has_embedded_truth = (
        est.get("truth_pos_ecef") is not None
        and np.asarray(est.get("truth_pos_ecef")).size > 0
    )
    if truth_file is None and not has_embedded_truth:
        vlog("Inferring truth file from dataset ...")
        m = re.search(r"X(\d+)", Path(args.dataset).stem)
        if m:
            dataset_id = m.group(1)
            candidates = [
                Path(f"STATE_X{dataset_id}.txt"),
                Path(f"STATE_X{dataset_id}_small.txt"),
            ]
            for cand in candidates:
                if cand.is_file():
                    truth_file = str(cand)
                    break
    if truth_file is None and not has_embedded_truth:
        raise FileNotFoundError(
            "Truth file not specified and could not be inferred from --dataset"
        )

    log_step("Assembling frames (ECEF/NED/Body) from estimate and truth ...")
    frames = assemble_frames(est, args.imu_file, args.gnss_file, truth_file)
    try:
        t_est, pos_est, vel_est, _ = frames["ECEF"]["fused"]
        truth_tuple = frames["ECEF"].get("truth")
        if truth_tuple is None:
            raise KeyError("truth")
        _, pos_truth, vel_truth, _ = truth_tuple
    except KeyError as exc:
        raise ValueError("Truth data required for ECEF residuals") from exc

    # Align time axis to start at zero for direct comparison with Task 6
    t_rel = t_est - t_est[0]
    log_step("Computing ECEF residuals (pos/vel/acc) ...")

    res_pos, res_vel, res_acc = compute_residuals(
        t_rel, pos_est, vel_est, pos_truth, vel_truth
    )

    out_dir = Path(args.output_dir)
    log_step("Plotting ECEF residual grids and norms ...")
    plot_residuals(
        t_rel,
        res_pos,
        res_vel,
        res_acc,
        args.dataset,
        args.gnss,
        args.method,
        out_dir,
    )

    # Optional: attitude angle error if truth file and quaternions are available
    log_step("Estimating attitude angle error (quaternions) ...")
    ang = compute_attitude_angle_error(est, truth_file, t_rel, pos_est, vel_est)
    if ang is None:
        print("[Task7] Attitude error plot skipped (no quaternions or truth file)")
    else:
        t_q, ang_deg = ang
        log_step("Plotting Task 7 attitude angle error and saving summary ...")
        pdf = plot_attitude_angle_error(t_q, ang_deg, args.dataset, args.gnss, args.method, out_dir)
        csv_path = save_angle_error_summary(ang_deg, args.dataset, args.gnss, args.method, out_dir)
        # Also produce a summary plot (bars + table) under Task 7.6 naming
        sum76 = plot_angle_error_summary(ang_deg, args.dataset, args.gnss, args.method, out_dir, subtask="6")
        print("Saved attitude angle error plot:", pdf.name)
        print("Saved attitude error summary:", csv_path.name)
        print("Saved Task 7.6 angle error summary plot:", sum76.name)

    # Task 7.6: quaternion and Euler plots truth vs estimate
    log_step("Computing truth-vs-est attitudes (quaternion/Euler) with time-offset alignment ...")
    att = compute_attitude_truth_vs_est(est, truth_file, t_rel, pos_est, vel_est)
    if att is None:
        print("[Task7.6] Quaternion/Euler plots skipped (no quaternions or truth file)")
    else:
        # Compute additional residual statistics to mirror MATLAB Task 7 output
        q_truth = att["q_truth_wxyz"]
        q_est = att["q_est_wxyz"]
        # Hemisphere alignment for component diffs
        q_est_aligned = _quat_sign_align(q_truth, q_est)
        dq = q_est_aligned - q_truth
        # Euler residuals (ZYX yaw, pitch, roll)
        e_err = _wrap_deg(att["eul_est_zyx_deg"] - att["eul_truth_zyx_deg"])
        # Quaternion angle error already available
        ang_deg = att["angle_deg"]
        # Error vector (body) from quaternion error
        q_err = _quat_error_wxyz(q_truth, q_est_aligned)
        err_vec_deg = _angle_axis_error_vec_deg(q_err)

        # Print summaries
        print(f"[Task7-Att] Rough time offset estimate (Truth -> KF) dt ≈ {att['time_offset']:+.3f} s")
        _print_table(
            "[Task7-Att] Euler residuals (deg):",
            ["Yaw", "Pitch", "Roll"],
            _stats_by_col(e_err, ["Yaw", "Pitch", "Roll"]),
        )
        _print_table(
            "[Task7-Att] Quaternion component residuals:",
            ["q_w", "q_x", "q_y", "q_z"],
            _stats_by_col(dq, ["q_w", "q_x", "q_y", "q_z"]),
        )
        s_ang = _stats("angle_deg", ang_deg)
        print(
            f"[Task7-Att] Quaternion angle error (deg): mean={s_ang['mean']:.3f} "
            f"std={s_ang['std']:.3f} rmse={s_ang['rmse']:.3f} max|.|={s_ang['max_abs']:.3f}"
        )
        _print_table(
            "[Task7-Att] Error vector (body, deg):",
            ["X_b", "Y_b", "Z_b"],
            _stats_by_col(err_vec_deg, ["X_b", "Y_b", "Z_b"]),
        )

        log_step("Plotting Task 7.6 quaternion/Euler truth-vs-est and residuals ...")
        qpdf = plot_quaternion_truth_vs_est(
            att["t_rel"], att["q_truth_wxyz"], att["q_est_wxyz"], args.dataset, args.gnss, args.method, out_dir
        )
        epdf = plot_euler_truth_vs_est(
            att["t_rel"], att["eul_truth_zyx_deg"], att["eul_est_zyx_deg"], args.dataset, args.gnss, args.method, out_dir
        )
        # Also save angle error under Task 7.6 naming
        qerr_pdf = plot_quaternion_component_error_task76(
            att["t_rel"], att["q_truth_wxyz"], att["q_est_wxyz"], args.dataset, args.gnss, args.method, out_dir
        )
        ang76_pdf = plot_attitude_angle_error_task76(
            att["t_rel"], att["angle_deg"], args.dataset, args.gnss, args.method, out_dir
        )
        eerr_pdf = plot_euler_error_task76(
            att["t_rel"], e_err, args.dataset, args.gnss, args.method, out_dir
        )
        # Save CSV summaries for residuals
        euler_csv = save_euler_error_summary_csv(e_err, args.dataset, args.gnss, args.method, out_dir)
        quat_csv = save_quat_component_error_summary_csv(dq, args.dataset, args.gnss, args.method, out_dir)
        # And a summary plot for Task 7.6
        sum76b = plot_angle_error_summary(att["angle_deg"], args.dataset, args.gnss, args.method, out_dir, subtask="6")
        print("Saved Task 7.6 quaternion plot:", qpdf.name)
        print("Saved Task 7.6 Euler plot:", epdf.name)
        print("Saved Task 7.6 quaternion error plot:", qerr_pdf.name)
        print("Saved Task 7.6 angle error plot:", ang76_pdf.name)
        print("Saved Task 7.6 Euler error plot:", eerr_pdf.name)
        print("Saved Task 7.6 Euler error summary:", euler_csv.name)
        print("Saved Task 7.6 quaternion component error summary:", quat_csv.name)
        print("Saved Task 7.6 angle error summary plot:", sum76b.name)


if __name__ == "__main__":
    main()
