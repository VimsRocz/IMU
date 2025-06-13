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


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------

def load_gnss_csv(path: str) -> pd.DataFrame:
    """Load GNSS data from CSV."""
    return pd.read_csv(path)


def load_imu_dat(path: str) -> np.ndarray:
    """Load IMU data from whitespace separated .dat file."""
    return np.loadtxt(path)


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

def run_pair(gnss_file: str, imu_file: str, label: str):
    """Run filter on one GNSS/IMU pair and return plotting data."""
    gnss = load_gnss_csv(gnss_file)
    _ = load_imu_dat(imu_file)  # IMU data currently unused

    lat = float(gnss.iloc[0].get("Latitude_deg", 0.0))
    lon = float(gnss.iloc[0].get("Longitude_deg", 0.0))
    C = compute_C_ECEF_to_NED(np.deg2rad(lat), np.deg2rad(lon))
    q = rot_to_quaternion(C.T)
    print(f"{label}: initial quaternion (body to NED):", q)

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
    yaw = np.degrees(np.arctan2(z[:, 4], z[:, 3]))
    pitch = np.zeros_like(yaw)
    roll = np.zeros_like(yaw)

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
    args = parser.parse_args()

    if args.all:
        pairs = [
            ("GNSS_X001.csv", "IMU_X001.dat", "clean"),
            ("GNSS_X002.csv", "IMU_X002.dat", "noisy"),
            ("GNSS_X002.csv", "IMU_X003.dat", "biased"),
        ]
    else:
        pairs = [(args.gnss_file, args.imu_file, "run")]

    results = [run_pair(g, i, l) for g, i, l in pairs]

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

