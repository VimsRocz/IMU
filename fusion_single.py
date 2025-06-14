# !/usr/bin/env python3
"""All-in-one IMU/GNSS fusion demo.

This script processes clean, noisy, and biased dataset pairs,
computes a simple Kalman filter using GNSS positions and velocities,
and writes PDF plots comparing the results.
"""

import argparse
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from imu_fusion.wahba import (
    davenport_q_method,
    svd_method,
)


def smooth_gnss(gnss: pd.DataFrame, window: int = 5) -> pd.DataFrame:
    """Return GNSS DataFrame smoothed with a moving average."""
    gnss = gnss.copy()
    cols = [
        "X_ECEF_m",
        "Y_ECEF_m",
        "Z_ECEF_m",
        "VX_ECEF_mps",
        "VY_ECEF_mps",
        "VZ_ECEF_mps",
    ]
    for c in cols:
        if c in gnss.columns:
            gnss[c] = (
                gnss[c]
                .rolling(window, center=True, min_periods=1)
                .mean()
            )
    return gnss


def remove_imu_bias(imu: np.ndarray, samples: int = 200) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Remove gyro and accelerometer bias estimated from first samples."""
    imu = imu.copy()
    gyro_bias = np.mean(imu[:samples, 2:5], axis=0)
    acc_bias = np.mean(imu[:samples, 5:8], axis=0)
    imu[:, 2:5] -= gyro_bias
    imu[:, 5:8] -= acc_bias
    return imu, gyro_bias, acc_bias


def _average_quaternions(qs: Sequence[np.ndarray]) -> np.ndarray:
    """Return average quaternion using the method of Markley."""
    A = np.zeros((4, 4))
    for q in qs:
        qn = q / np.linalg.norm(q)
        A += np.outer(qn, qn)
    eigvals, eigvecs = np.linalg.eigh(A)
    q_avg = eigvecs[:, np.argmax(eigvals)]
    if q_avg[0] < 0:
        q_avg = -q_avg
    return q_avg


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------

def load_gnss_csv(path: str) -> pd.DataFrame:
    """Load GNSS data from CSV."""
    return pd.read_csv(path)


def load_imu_dat(path: str) -> np.ndarray:
    """Load IMU data from whitespace separated .dat file."""
    return np.loadtxt(path)


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Approximate conversion from ECEF to geodetic coordinates."""
    a = 6378137.0
    e_sq = 6.69437999014e-3
    p = np.sqrt(x ** 2 + y ** 2)
    theta = np.arctan2(z * a, p * (1 - e_sq))
    lon = np.arctan2(y, x)
    lat = np.arctan2(z + e_sq * a * np.sin(theta) ** 3, p - e_sq * a * np.cos(theta) ** 3)
    N = a / np.sqrt(1 - e_sq * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N
    return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)


# ---------------------------------------------------------------------------
# Attitude helpers
# ---------------------------------------------------------------------------

def compute_C_ECEF_to_NED(lat: float, lon: float) -> np.ndarray:
    """Rotation matrix from ECEF to NED."""
    sin_phi = np.sin(lat)
    cos_phi = np.cos(lat)
    sin_lambda = np.sin(lon)
    cos_lambda = np.cos(lon)
    return np.array([
        [-sin_phi * cos_lambda, -sin_phi * sin_lambda, cos_phi],
        [-sin_lambda, cos_lambda, 0.0],
        [-cos_phi * cos_lambda, -cos_phi * sin_lambda, -sin_phi],
    ])


def rot_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to quaternion."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)


def triad(v1_b: np.ndarray, v2_b: np.ndarray, v1_n: np.ndarray, v2_n: np.ndarray) -> np.ndarray:
    """Compute body-to-NED rotation using the TRIAD method."""
    t1_b = v1_b / np.linalg.norm(v1_b)
    t2_b = np.cross(t1_b, v2_b)
    t2_b = t2_b / np.linalg.norm(t2_b)
    t3_b = np.cross(t1_b, t2_b)

    t1_n = v1_n / np.linalg.norm(v1_n)
    t2_n = np.cross(t1_n, v2_n)
    t2_n = t2_n / np.linalg.norm(t2_n)
    t3_n = np.cross(t1_n, t2_n)

    return np.column_stack((t1_n, t2_n, t3_n)) @ np.column_stack((t1_b, t2_b, t3_b)).T


def estimate_initial_orientation(gnss: pd.DataFrame, imu: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Estimate initial body-to-NED rotation matrix from first samples."""
    # Determine reference latitude/longitude from first valid GNSS row
    valid = gnss[(gnss["X_ECEF_m"] != 0) | (gnss["Y_ECEF_m"] != 0) | (gnss["Z_ECEF_m"] != 0)]
    row = valid.iloc[0]
    lat_deg, lon_deg, _ = ecef_to_geodetic(row["X_ECEF_m"], row["Y_ECEF_m"], row["Z_ECEF_m"])

    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)

    g_ned = np.array([0.0, 0.0, 9.81])
    omega_e = 7.2921159e-5
    omega_ned = omega_e * np.array([np.cos(lat), 0.0, -np.sin(lat)])

    dt_imu = 1.0 / 400.0
    acc = imu[:, 5:8] / dt_imu
    gyro = imu[:, 2:5] / dt_imu
    N_static = min(4000, len(imu))
    acc_mean = np.mean(acc[:N_static], axis=0)
    gyro_mean = np.mean(gyro[:N_static], axis=0)
    g_body = -acc_mean
    omega_body = gyro_mean

    C_b_n = triad(g_body, omega_body, g_ned, omega_ned)
    return C_b_n, lat_deg, lon_deg


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to roll, pitch, yaw in degrees."""
    w, x, y, z = q
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return np.degrees([roll, pitch, yaw])


# ---------------------------------------------------------------------------
# Kalman filter
# ---------------------------------------------------------------------------

def kalman_with_residuals(
    zs: np.ndarray, F: np.ndarray, H: np.ndarray, Q: np.ndarray, R: np.ndarray, x0: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Run Kalman filter and return states and residuals."""
    kf = KalmanFilter(dim_x=len(x0), dim_z=zs.shape[1])
    kf.x = x0
    kf.F = F
    kf.H = H
    kf.Q = Q
    kf.R = R
    xs = []
    residuals = []
    for z in zs:
        kf.predict()
        residual = z - (kf.H @ kf.x)
        kf.update(z)
        xs.append(kf.x.copy())
        residuals.append(residual)
    return np.array(xs), np.array(residuals)


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def _setup_plot(title: str, xlabel: str, ylabel: str) -> None:
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)


def plot_ned_positions(time: Iterable, ned: Iterable, label: str, title: str, outfile: str) -> None:
    plt.figure()
    for i, comp in enumerate("NED"):
        plt.plot(time, [r[i] for r in ned], label=f"{label} {comp}")
    _setup_plot(title, "Time (s)", "Position (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_residuals(time: Iterable, residuals: Sequence[Sequence[float]], title: str, outfile: str) -> None:
    plt.figure(figsize=(8, 6))
    res = list(residuals)
    comps = ["North", "East", "Down"]
    for i in range(3):
        plt.plot(time, [r[i] for r in res], label=f"pos {comps[i]}")
    for i in range(3):
        plt.plot(time, [r[i + 3] for r in res], linestyle="--", label=f"vel {comps[i]}")
    _setup_plot(title, "Time (s)", "Residual")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_attitude(time: Iterable, yaw: Sequence[float], pitch: Sequence[float], roll: Sequence[float], title: str, outfile: str) -> None:
    plt.figure()
    plt.plot(time, roll, label="Roll")
    plt.plot(time, pitch, label="Pitch")
    plt.plot(time, yaw, label="Yaw")
    _setup_plot(title, "Time (s)", "Angle (deg)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_ned_positions_multi(times: Sequence[Iterable], neds: Sequence[Iterable], labels: Sequence[str], title: str, outfile: str) -> None:
    plt.figure()
    for t, ned, label in zip(times, neds, labels):
        for i, comp in enumerate("NED"):
            plt.plot(t, [r[i] for r in ned], label=f"{label} {comp}")
    _setup_plot(title, "Time (s)", "Position (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_residuals_multi(times: Sequence[Iterable], residual_sets: Sequence[Sequence[Sequence[float]]], labels: Sequence[str], title: str, outfile: str) -> None:
    plt.figure(figsize=(8, 6))
    comps = ["North", "East", "Down"]
    for t, residuals, label in zip(times, residual_sets, labels):
        res = list(residuals)
        for i in range(3):
            plt.plot(t, [r[i] for r in res], label=f"{label} pos {comps[i]}")
        for i in range(3):
            plt.plot(t, [r[i + 3] for r in res], linestyle="--", label=f"{label} vel {comps[i]}")
    _setup_plot(title, "Time (s)", "Residual")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_attitude_multi(times: Sequence[Iterable], yaws: Sequence[Iterable], pitches: Sequence[Iterable], rolls: Sequence[Iterable], labels: Sequence[str], title: str, outfile: str) -> None:
    plt.figure()
    for t, yaw, pitch, roll, label in zip(times, yaws, pitches, rolls, labels):
        plt.plot(t, roll, label=f"{label} Roll")
        plt.plot(t, pitch, label=f"{label} Pitch")
        plt.plot(t, yaw, label=f"{label} Yaw")
    _setup_plot(title, "Time (s)", "Angle (deg)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


# ---------------------------------------------------------------------------
# Filter run helpers
# ---------------------------------------------------------------------------

def _initial_quaternion(gnss: pd.DataFrame, imu: np.ndarray, method: str) -> np.ndarray:
    """Return initial body-to-NED quaternion using the chosen Wahba method."""
    lat = np.deg2rad(float(gnss.iloc[0].get("Latitude_deg", 0.0)))
    g_ned = np.array([0.0, 0.0, 9.81])
    omega_ie = 7.2921159e-5 * np.array([np.cos(lat), 0.0, -np.sin(lat)])

    dt = 1.0 / 400.0
    acc = imu[:, 5:8] / dt
    gyro = imu[:, 2:5] / dt
    N = min(4000, len(acc))
    g_body = -np.mean(acc[:N], axis=0)
    omega_body = np.mean(gyro[:N], axis=0)

    if method == "TRIAD":
        R = triad(g_body, omega_body, g_ned, omega_ie)
        return rot_to_quaternion(R)
    if method == "Davenport":
        R = davenport_q_method(g_body, omega_body, g_ned, omega_ie)
        return rot_to_quaternion(R)
    if method == "SVD":
        R = svd_method(g_body, omega_body, g_ned, omega_ie)
        return rot_to_quaternion(R)
    if method == "MEAN":
        qs = [
            rot_to_quaternion(triad(g_body, omega_body, g_ned, omega_ie)),
            rot_to_quaternion(davenport_q_method(g_body, omega_body, g_ned, omega_ie)),
            rot_to_quaternion(svd_method(g_body, omega_body, g_ned, omega_ie)),
        ]
        return _average_quaternions(qs)
    # fallback to SVD if unknown method
    R = svd_method(g_body, omega_body, g_ned, omega_ie)
    return rot_to_quaternion(R)


def run_pair(
    gnss_file: str,
    imu_file: str,
    label: str,
    method: str = "TRIAD",
    smooth_window: int = 5,
    bias_samples: int = 200,
):
    """Run filter on one GNSS/IMU pair and return plotting data."""
    gnss_raw = load_gnss_csv(gnss_file)
    imu_raw = load_imu_dat(imu_file)

    gnss = smooth_gnss(gnss_raw, smooth_window)
    imu, gyro_bias, acc_bias = remove_imu_bias(imu_raw, bias_samples)
    print(f"{label}: gyro bias {gyro_bias}, acc bias {acc_bias}")

    q = _initial_quaternion(gnss, imu, method)
    lat_deg = float(gnss.iloc[0].get("Latitude_deg", 0.0))
    lon_deg = float(gnss.iloc[0].get("Longitude_deg", 0.0))
    print(
        f"{label}: {method} initial lat {lat_deg:.4f} deg lon {lon_deg:.4f} deg"
    )
    print(f"{label}: {method} quaternion (body to NED):", q)

    if {"X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m", "VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"} <= set(gnss.columns):
        z = gnss[[
            "X_ECEF_m",
            "Y_ECEF_m",
            "Z_ECEF_m",
            "VX_ECEF_mps",
            "VY_ECEF_mps",
            "VZ_ECEF_mps",
        ]].values
    else:
        z = np.zeros((len(gnss), 6))

    times = gnss["Posix_Time"].values
    dt = float(np.mean(np.diff(times))) if len(times) > 1 else 1.0

    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    H = np.eye(6)
    Q = np.eye(6) * 0.01
    R = np.eye(6) * 0.1
    xs, residuals = kalman_with_residuals(z, F, H, Q, R, z[0])

    time_rel = times - times[0]
    roll, pitch, yaw = quat_to_euler(q)
    roll = np.full_like(time_rel, roll)
    pitch = np.full_like(time_rel, pitch)
    yaw = np.full_like(time_rel, yaw)

    return {
        "label": label,
        "time": time_rel,
        "pos": xs[:, :3],
        "res": residuals,
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
    }


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="All-in-one IMU/GNSS fusion demo")
    parser.add_argument("--all", action="store_true", help="run all dataset combinations")
    parser.add_argument("--gnss-file", default="GNSS_X001.csv", help="GNSS CSV file")
    parser.add_argument("--imu-file", default="IMU_X001.dat", help="IMU dat file")
    parser.add_argument(
        "--init-method",
        choices=["TRIAD", "Davenport", "SVD", "MEAN", "ALL"],
        default="TRIAD",
        help="Attitude initialization method",
    )
    parser.add_argument("--smooth-window", type=int, default=5, help="GNSS moving average window")
    parser.add_argument("--bias-samples", type=int, default=200, help="IMU samples used for bias estimation")
    args = parser.parse_args()

    if args.all:
        pairs = [
            ("GNSS_X001.csv", "IMU_X001.dat", "clean"),
            ("GNSS_X002.csv", "IMU_X002.dat", "noisy"),
            ("GNSS_X002.csv", "IMU_X003.dat", "biased"),
        ]
    else:
        pairs = [(args.gnss_file, args.imu_file, "run")]

    methods = [args.init_method]
    if args.init_method == "ALL":
        methods = ["TRIAD", "Davenport", "SVD", "MEAN"]

    results = []
    for g, i, l in pairs:
        for m in methods:
            results.append(
                run_pair(
                    g,
                    i,
                    f"{l}_{m}",
                    m,
                    smooth_window=args.smooth_window,
                    bias_samples=args.bias_samples,
                )
            )

    for r in results:
        base = r["label"]
        plot_ned_positions(r["time"], r["pos"], base, f"Position {base}", f"positions_{base}.pdf")
        plot_residuals(r["time"], r["res"], f"Residuals {base}", f"residuals_{base}.pdf")
        plot_attitude(r["time"], r["yaw"], r["pitch"], r["roll"], f"Attitude {base}", f"attitude_{base}.pdf")

    if len(results) > 1:
        times = [r["time"] for r in results]
        poss = [r["pos"] for r in results]
        ress = [r["res"] for r in results]
        yaws = [r["yaw"] for r in results]
        pitches = [r["pitch"] for r in results]
        rolls = [r["roll"] for r in results]
        labels = [r["label"] for r in results]

        plot_ned_positions_multi(times, poss, labels, "Position Comparison", "positions_compare.pdf")
        plot_residuals_multi(times, ress, labels, "Residual Comparison", "residuals_compare.pdf")
        plot_attitude_multi(times, yaws, pitches, rolls, labels, "Attitude Comparison", "attitude_compare.pdf")

    print("Plots written for datasets:", ", ".join([r["label"] for r in results]))


if __name__ == "__main__":
    main()

