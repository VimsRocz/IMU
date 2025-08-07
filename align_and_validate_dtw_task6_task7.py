# Updated script for aligning IMU/GNSS estimate with truth using DTW
# and generating Task 6/7 plots. Designed for IMU_X001_GNSS_X001_TRIAD
# filter output. Estimator and truth logs may have different time bases;
# ``summarize_timebase`` prints their ranges and effective sample rates.

import argparse
import os
import subprocess

import matplotlib.pyplot as plt
import numpy as np
from fastdtw import fastdtw
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
from scipy.spatial.distance import euclidean


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def smooth_data(data: np.ndarray, window_length: int = 7, polyorder: int = 2) -> np.ndarray:
    """Apply Savitzky-Golay filter to reduce noise."""
    if len(data) < window_length:
        return data
    return savgol_filter(data, window_length, polyorder, axis=0)


def load_estimator_data(path: str):
    """Return position, velocity, quaternion, time and reference parameters."""
    data = np.load(path, allow_pickle=True)
    try:
        pos = data["pos_ned_m"]
        vel = data["vel_ned_ms"]
        quat = data["att_quat"]
        time = data["time_s"]
        ref_lat = data["ref_lat_rad"]
        ref_lon = data["ref_lon_rad"]
        ref_r0 = data["ref_r0_m"]
    except KeyError as exc:  # pragma: no cover - debugging helper
        print(f"Missing key {exc} in {path}. Available keys: {list(data.keys())}")
        raise
    print("Using 'pos_ned_m' for position, 'vel_ned_ms' for velocity, 'att_quat' for quaternion")
    return pos, vel, quat, time, ref_lat, ref_lon, ref_r0


def load_truth_data(path: str):
    """Return position and time from ``STATE_X*.txt`` truth logs.

    The text files start with a sample counter followed by the timestamp and
    ECEF coordinates.  Earlier versions of this helper mistakenly treated the
    counter as the time column, leading to misaligned overlays when the first
    column was used.  We now explicitly skip the counter and extract the time
    from column 1 and the position from columns 2–4.
    """
    data = np.loadtxt(path)
    time = data[:, 1]
    pos = data[:, 2:5]
    frame = "ned" if np.abs(pos).max() < 1e6 else "ecef"
    return pos, time, frame


def summarize_timebase(label: str, time: np.ndarray) -> None:
    """Print basic timing statistics for a dataset.

    Parameters
    ----------
    label : str
        Identifier for the dataset (e.g. ``"Estimator"``).
    time : np.ndarray
        Time vector in seconds.
    """
    if time.size < 2:
        print(f"{label} time vector too short for statistics")
        return
    dt = np.diff(time)
    mean_dt = np.mean(dt)
    freq = 1.0 / mean_dt if mean_dt > 0 else float('inf')
    print(
        f"{label} time start={time[0]:.3f}s end={time[-1]:.3f}s "
        f"mean_dt={mean_dt:.3f}s ({freq:.2f} Hz)"
    )


def ned_to_ecef(pos_ned: np.ndarray, lat: float, lon: float, r0: np.ndarray) -> np.ndarray:
    """Convert NED positions to ECEF using reference lat/lon/r0."""
    C = np.array(
        [
            [-np.sin(lat) * np.cos(lon), -np.sin(lon), -np.cos(lat) * np.cos(lon)],
            [-np.sin(lat) * np.sin(lon), np.cos(lon), -np.cos(lat) * np.sin(lon)],
            [np.cos(lat), 0.0, -np.sin(lat)],
        ]
    )
    return (r0 + C.T @ pos_ned.T).T


def quat_to_dcm(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion [w,x,y,z] to a DCM."""
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)],
        ]
    )


# ---------------------------------------------------------------------------
# Plot generation
# ---------------------------------------------------------------------------

def plot_task6_results(
    est_pos: np.ndarray,
    est_vel: np.ndarray,
    est_quat: np.ndarray,
    est_time: np.ndarray,
    truth_pos: np.ndarray,
    truth_time: np.ndarray,
    ref_lat: float,
    ref_lon: float,
    ref_r0: np.ndarray,
    output_dir: str,
    method_name: str,
) -> None:
    """Create and save Task 6 plots matching Task 5 layout."""

    truth_interp = interp1d(truth_time, truth_pos, axis=0, bounds_error=False, fill_value="extrapolate")
    truth_pos_i = truth_interp(est_time)

    dt = est_time[1] - est_time[0] if len(est_time) > 1 else 1.0
    est_acc = np.diff(est_vel, axis=0) / dt
    est_acc = np.vstack([est_acc, est_acc[-1]])

    # Plot 1 - NED position/velocity/acceleration
    plt.figure(figsize=(15, 10))
    labels = ["North", "East", "Down"]
    for i, lab in enumerate(labels):
        plt.subplot(3, 3, i + 1)
        plt.plot(est_time, est_pos[:, i], label="Fused")
        plt.plot(est_time, truth_pos_i[:, i], "--", label="Truth")
        plt.xlabel("Time (s)")
        plt.ylabel(f"Position {lab} (m)")
        plt.legend(); plt.grid(True)
    for i, lab in enumerate(labels):
        plt.subplot(3, 3, i + 4)
        plt.plot(est_time, est_vel[:, i], label="Fused")
        plt.xlabel("Time (s)"); plt.ylabel(f"Velocity {lab} (m/s)")
        plt.legend(); plt.grid(True)
    for i, lab in enumerate(labels):
        plt.subplot(3, 3, i + 7)
        plt.plot(est_time, est_acc[:, i], label="Fused")
        plt.xlabel("Time (s)"); plt.ylabel(f"Acceleration {lab} (m/s²)")
        plt.legend(); plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(output_dir, f"{method_name}_task6_results_ned.pdf")
    plt.savefig(out_path)
    plt.close()

    # Plot 2 - NED position & body-frame velocity
    vel_body = np.zeros_like(est_vel)
    for i, q in enumerate(est_quat):
        vel_body[i] = quat_to_dcm(q).T @ est_vel[i]
    plt.figure(figsize=(15, 10))
    for i, lab in enumerate(labels):
        plt.subplot(2, 3, i + 1)
        plt.plot(est_time, est_pos[:, i], label="Fused")
        plt.plot(est_time, truth_pos_i[:, i], "--", label="Truth")
        plt.xlabel("Time (s)"); plt.ylabel(f"Position {lab} (m)")
        plt.legend(); plt.grid(True)
    body_labels = ["X", "Y", "Z"]
    for i, lab in enumerate(body_labels):
        plt.subplot(2, 3, i + 4)
        plt.plot(est_time, vel_body[:, i], label="Fused")
        plt.xlabel("Time (s)"); plt.ylabel(f"Velocity {lab} (m/s)")
        plt.legend(); plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(output_dir, f"{method_name}_task6_results_mixed.pdf")
    plt.savefig(out_path)
    plt.close()

    # Plot 3 - All data in NED frame
    plt.figure(figsize=(15, 5))
    for i, lab in enumerate(labels):
        plt.subplot(1, 3, i + 1)
        plt.plot(est_time, est_pos[:, i], label="Fused")
        plt.plot(est_time, truth_pos_i[:, i], "--", label="Truth")
        plt.xlabel("Time (s)"); plt.ylabel(f"Position {lab} (m)")
        plt.legend(); plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(output_dir, f"{method_name}_task6_results_all_ned.pdf")
    plt.savefig(out_path)
    plt.close()

    # Plot 4 - All data in ECEF frame
    est_ecef = ned_to_ecef(est_pos, ref_lat, ref_lon, ref_r0)
    truth_ecef = ned_to_ecef(truth_pos_i, ref_lat, ref_lon, ref_r0)
    plt.figure(figsize=(15, 5))
    ecef_labels = ["X", "Y", "Z"]
    for i, lab in enumerate(ecef_labels):
        plt.subplot(1, 3, i + 1)
        plt.plot(est_time, est_ecef[:, i], label="Fused")
        plt.plot(est_time, truth_ecef[:, i], "--", label="Truth")
        plt.xlabel("Time (s)"); plt.ylabel(f"Position {lab} (m)")
        plt.legend(); plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(output_dir, f"{method_name}_task6_results_all_ecef.pdf")
    plt.savefig(out_path)
    plt.close()

    # Plot 5 - All data in body frame
    pos_body = np.zeros_like(est_pos)
    for i, q in enumerate(est_quat):
        pos_body[i] = quat_to_dcm(q).T @ est_pos[i]
    plt.figure(figsize=(15, 5))
    for i, lab in enumerate(body_labels):
        plt.subplot(1, 3, i + 1)
        plt.plot(est_time, pos_body[:, i], label="Fused")
        plt.xlabel("Time (s)"); plt.ylabel(f"Position {lab} (m)")
        plt.legend(); plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(output_dir, f"{method_name}_task6_results_all_body.pdf")
    plt.savefig(out_path)
    plt.close()


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Align and validate datasets using DTW")
    ap.add_argument("--est-file", required=True, help="Path to *_kf_output.npz file")
    ap.add_argument("--truth-file", required=True, help="Path to STATE_X001.txt file")
    ap.add_argument("--output-dir", default="results", help="Directory for outputs")
    ap.add_argument("--duration", type=float, default=1250.0, help="Expected duration")
    args = ap.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    est_pos, est_vel, est_quat, est_time, ref_lat, ref_lon, ref_r0 = load_estimator_data(args.est_file)
    truth_pos, truth_time, truth_frame = load_truth_data(args.truth_file)

    summarize_timebase("Estimator", est_time)
    summarize_timebase("Truth", truth_time)

    if truth_frame != "ned":
        raise RuntimeError("Truth trajectory must be in NED frame")

    # Smooth for alignment
    est_pos = smooth_data(est_pos)
    truth_pos = smooth_data(truth_pos)

    eps = 1e-10
    est_norm = (est_pos - np.median(est_pos, axis=0)) / (np.std(est_pos, axis=0) + eps)
    truth_norm = (truth_pos - np.median(truth_pos, axis=0)) / (np.std(truth_pos, axis=0) + eps)

    print("Starting Task 6 overlay ...")
    distance, path = fastdtw(est_norm, truth_norm, dist=euclidean)
    est_idx, truth_idx = zip(*path)

    n = min(len(est_idx), len(truth_idx), len(est_time))
    common_time = np.linspace(0.0, args.duration, n)
    est_interp = interp1d(np.linspace(0.0, args.duration, len(est_idx)), est_pos[list(est_idx)], axis=0)
    truth_interp = interp1d(np.linspace(0.0, args.duration, len(truth_idx)), truth_pos[list(truth_idx)], axis=0)
    est_vel_i = interp1d(np.linspace(0.0, args.duration, len(est_idx)), est_vel[list(est_idx)], axis=0)
    est_quat_i = interp1d(np.linspace(0.0, args.duration, len(est_idx)), est_quat[list(est_idx)], axis=0)

    aligned_est_pos = est_interp(common_time)
    aligned_truth_pos = truth_interp(common_time)
    aligned_est_vel = est_vel_i(common_time)
    aligned_est_quat = est_quat_i(common_time)

    aligned_est_file = os.path.join(args.output_dir, "aligned_" + os.path.basename(args.est_file))
    np.savez(
        aligned_est_file,
        time_s=common_time,
        pos_ned_m=aligned_est_pos,
        vel_ned_ms=aligned_est_vel,
        att_quat=aligned_est_quat,
        ref_lat_rad=ref_lat,
        ref_lon_rad=ref_lon,
        ref_r0_m=ref_r0,
    )

    aligned_truth_file = os.path.join(args.output_dir, "aligned_" + os.path.basename(args.truth_file))
    np.savetxt(
        aligned_truth_file,
        np.column_stack((common_time, aligned_truth_pos)),
        fmt="%.6f",
        header="time x y z",
    )

    method_name = os.path.basename(args.est_file).replace("_kf_output.npz", "")
    plot_task6_results(
        aligned_est_pos,
        aligned_est_vel,
        aligned_est_quat,
        common_time,
        truth_pos,
        truth_time,
        ref_lat,
        ref_lon,
        ref_r0,
        args.output_dir,
        method_name,
    )

    print("Running Task 7 evaluation ...")
    cmd = [
        "python",
        "src/validate_3sigma.py",
        "--est-file",
        aligned_est_file,
        "--truth-file",
        aligned_truth_file,
        "--output-dir",
        args.output_dir,
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    print("Validation Output:")
    print(res.stdout)
    print(res.stderr)

    pos_err = np.linalg.norm(aligned_est_pos - aligned_truth_pos, axis=1)
    print(f"Aligned time range: 0 to {common_time[-1]:.2f} seconds")
    print(f"Position errors: mean={np.mean(pos_err):.4f} m, std={np.std(pos_err):.4f} m")


if __name__ == "__main__":
    main()
