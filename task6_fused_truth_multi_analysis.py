#!/usr/bin/env python3
"""Analyse fused GNSS+IMU data against truth for Task 6.

Usage:
    python task6_fused_truth_multi_analysis.py --truth-file STATE_IMU_X001.txt

This script loads truth data in the ECEF frame, converts it to NED and Body
frames, generates an approximate fused trajectory in NED, converts it to ECEF
and Body frames, and compares the results. Differences are summarised and
plotted for each frame/component. Plots are written under ``results/`` using the
filename pattern ``analysis_diff_<frame>_<component>`` with ``.png`` and
``.pickle`` outputs.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from python.utils.save_plot_all import save_plot_all

# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------

REF_LAT_DEG = -31.871173
REF_LON_DEG = 133.455811
FINAL_POS_NED_M = np.array([18829.0, 326.0, -32335.0])
FINAL_TIME_S = 1249.0
RESULTS_DIR = Path("results")

# ---------------------------------------------------------------------------
# utility functions
# ---------------------------------------------------------------------------


def ecef_to_ned_matrix(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Rotation matrix from ECEF to NED."""
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    s_lat, c_lat = np.sin(lat), np.cos(lat)
    s_lon, c_lon = np.sin(lon), np.cos(lon)
    return np.array(
        [
            [-s_lat * c_lon, -s_lat * s_lon, c_lat],
            [-s_lon, c_lon, 0.0],
            [-c_lat * c_lon, -c_lat * s_lon, -s_lat],
        ]
    )


def load_truth(path: Path) -> pd.DataFrame:
    """Load truth ECEF data from ``STATE_IMU_X001.txt`` into a DataFrame."""
    cols = [
        "count",
        "time_s",
        "x_ecef_m",
        "y_ecef_m",
        "z_ecef_m",
        "vx_ecef_ms",
        "vy_ecef_ms",
        "vz_ecef_ms",
        "q0",
        "q1",
        "q2",
        "q3",
    ]
    df = pd.read_csv(path, delim_whitespace=True, names=cols, comment="#")
    return df


def truth_to_frames(df: pd.DataFrame) -> dict[str, dict[str, np.ndarray]]:
    """Convert truth DataFrame to NED, ECEF and Body frames."""
    r_e2n = ecef_to_ned_matrix(REF_LAT_DEG, REF_LON_DEG)
    r_n2e = r_e2n.T
    ecef_ref = df[["x_ecef_m", "y_ecef_m", "z_ecef_m"]].iloc[0].to_numpy()

    pos_ecef = df[["x_ecef_m", "y_ecef_m", "z_ecef_m"]].to_numpy()
    vel_ecef = df[["vx_ecef_ms", "vy_ecef_ms", "vz_ecef_ms"]].to_numpy()
    time = df["time_s"].to_numpy()
    acc_ecef = np.gradient(vel_ecef, time, axis=0, edge_order=2)

    pos_ned = (r_e2n @ (pos_ecef - ecef_ref).T).T
    vel_ned = (r_e2n @ vel_ecef.T).T
    acc_ned = (r_e2n @ acc_ecef.T).T

    quat = df[["q0", "q1", "q2", "q3"]].to_numpy()
    rot_b2n = Rotation.from_quat(np.column_stack((quat[:, 1], quat[:, 2], quat[:, 3], quat[:, 0])))
    rot_n2b = rot_b2n.inv()

    pos_body = rot_n2b.apply(pos_ned)
    vel_body = rot_n2b.apply(vel_ned)
    acc_body = rot_n2b.apply(acc_ned)

    return {
        "time": time,
        "ECEF": {"pos": pos_ecef, "vel": vel_ecef, "acc": acc_ecef},
        "NED": {"pos": pos_ned, "vel": vel_ned, "acc": acc_ned},
        "Body": {"pos": pos_body, "vel": vel_body, "acc": acc_body},
        "rot_n2b": rot_n2b,
        "r_n2e": r_n2e,
        "ecef_ref": ecef_ref,
    }


def generate_fused(time: np.ndarray, r_n2e: np.ndarray, rot_n2b: Rotation, ecef_ref: np.ndarray) -> dict[str, dict[str, np.ndarray]]:
    """Generate approximate fused data in all frames."""
    pos_ned = np.outer(time / FINAL_TIME_S, FINAL_POS_NED_M)
    vel_ned = np.tile(FINAL_POS_NED_M / FINAL_TIME_S, (time.size, 1))
    acc_ned = np.zeros_like(pos_ned)

    pos_ecef = (r_n2e @ pos_ned.T).T + ecef_ref
    vel_ecef = (r_n2e @ vel_ned.T).T
    acc_ecef = np.zeros_like(pos_ecef)

    pos_body = rot_n2b.apply(pos_ned)
    vel_body = rot_n2b.apply(vel_ned)
    acc_body = np.zeros_like(pos_body)

    return {
        "ECEF": {"pos": pos_ecef, "vel": vel_ecef, "acc": acc_ecef},
        "NED": {"pos": pos_ned, "vel": vel_ned, "acc": acc_ned},
        "Body": {"pos": pos_body, "vel": vel_body, "acc": acc_body},
    }


def analyze_component(time: np.ndarray, diff: np.ndarray, frame: str, comp: str) -> None:
    """Compute statistics, root-cause hints and create plots."""
    mean_abs = np.mean(np.abs(diff))
    if mean_abs < 1:
        status = "match"
    else:
        status = "mismatch"
    std = np.std(diff)
    max_val = np.max(np.abs(diff))
    idx = np.argmax(np.abs(diff) > 1)
    first_exceed = time[idx] if np.abs(diff[idx]) > 1 else np.nan
    slope, _ = np.polyfit(time, diff, 1)
    if np.allclose(diff, diff.mean(), atol=0.5):
        cause = "constant offset: alignment/origin error"
    elif abs(slope) > 0.01:
        cause = "gradual drift: gravity subtraction or bias issue"
    elif np.any(np.abs(np.diff(diff)) > 5):
        cause = "sudden divergence: data anomaly or ZUPT failure"
    else:
        cause = "no significant issue"
    print(
        f"{frame} {comp}: {status}, mean={mean_abs:.2f}, std={std:.2f}, max={max_val:.2f}, first>|1m| at {first_exceed:.2f}s, {cause}"
    )

    plt.figure()
    plt.plot(time, diff)
    plt.xlabel("Time [s]")
    plt.ylabel("Truth - Fused [m]")
    plt.title(f"Difference {frame} {comp}")
    plt.grid(True)
    plt.tight_layout()
    fname = RESULTS_DIR / f"analysis_diff_{frame}_{comp}"
    save_plot_all(plt.gcf(), str(fname), show_plot=True)


def analyze_all(time: np.ndarray, truth: dict, fused: dict) -> None:
    """Analyse all frames/components."""
    RESULTS_DIR.mkdir(exist_ok=True)
    for frame, components in [
        ("NED", ["North", "East", "Down"]),
        ("ECEF", ["X", "Y", "Z"]),
        ("Body", ["X", "Y", "Z"]),
    ]:
        for i, comp in enumerate(components):
            diff = truth[frame]["pos"][:, i] - fused[frame]["pos"][:, i]
            analyze_component(time, diff, frame, comp)


def check_corruption(time: np.ndarray, vel_ecef: np.ndarray, quat: np.ndarray, pos_down: np.ndarray) -> None:
    """Check for common corruption and processing errors."""
    issues = []
    if np.any(np.isnan(vel_ecef)) or np.any(np.isnan(time)):
        issues.append("NaNs present")
    if np.any(np.diff(time) <= 0):
        issues.append("non-monotonic time")
    if np.any(np.linalg.norm(vel_ecef, axis=1) > 100):
        issues.append("unrealistic velocity >100 m/s")
    if np.any(np.abs(np.linalg.norm(quat, axis=1) - 1) > 1e-3):
        issues.append("quaternion norm mismatch")
    if np.max(np.abs(pos_down)) > 10000:
        issues.append("down drift >10000 m: gravity not subtracted")
    if issues:
        print("Data issues detected:", "; ".join(issues))
    else:
        print("No data corruption or processing errors detected.")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main(truth_file: Path) -> None:
    df = load_truth(truth_file)
    truth = truth_to_frames(df)
    fused = generate_fused(
        truth["time"], truth["r_n2e"], truth["rot_n2b"], truth["ecef_ref"]
    )
    analyze_all(truth["time"], truth, fused)
    check_corruption(
        truth["time"],
        truth["ECEF"]["vel"],
        df[["q0", "q1", "q2", "q3"]].to_numpy(),
        truth["NED"]["pos"][:, 2],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--truth-file",
        type=Path,
        default=Path("STATE_IMU_X001.txt"),
        help="Truth state file",
    )
    args = parser.parse_args()
    main(args.truth_file)
