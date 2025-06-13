#!/usr/bin/env python3
"""Command line entry point for IMU/GNSS fusion demo."""
import argparse
import numpy as np

from imu_fusion.data import load_gnss_csv, load_imu_dat
from imu_fusion.attitude import compute_C_ECEF_to_NED, rot_to_quaternion
from imu_fusion.kalman import kalman_with_residuals
from imu_fusion.plotting import (
    plot_ned_positions,
    plot_residuals,
    plot_attitude,
    plot_ned_positions_multi,
    plot_residuals_multi,
    plot_attitude_multi,
)


def run_pair(gnss_file: str, imu_file: str, label: str):
    """Run filter on one GNSS/IMU pair and return plotting data."""
    gnss = load_gnss_csv(gnss_file)
    _ = load_imu_dat(imu_file)  # IMU data loaded but not used in this simple demo

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


def main() -> None:
    parser = argparse.ArgumentParser(description="IMU/GNSS fusion demo")
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
