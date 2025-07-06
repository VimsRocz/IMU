#!/usr/bin/env python3
"""Compare fusion output with ground truth using local gravity.

This script computes the gravity magnitude from ``STATE_X001.txt`` and
validates the ECEF position trajectories produced by each fusion method.
Switch between 'fusion' (compare estimator vs truth) and 'truth'
(plot only the truth trajectory) using ``--mode``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyproj import Transformer
import scipy.io as sio

from src.utils import normal_gravity


# ---------------------------------------------------------------------------
# Gravity computation
# ---------------------------------------------------------------------------

def compute_gravity_from_state(state_file: Path) -> tuple[float, float, float, float]:
    """Return gravity magnitude [m/s^2] using the STATE trajectory."""
    cols = [
        "count",
        "time",
        "X_ECEF_m",
        "Y_ECEF_m",
        "Z_ECEF_m",
        "VX_ECEF_mps",
        "VY_ECEF_mps",
        "VZ_ECEF_mps",
        "q0",
        "q1",
        "q2",
        "q3",
    ]
    df = pd.read_csv(state_file, sep="\s+", comment="#", names=cols)
    x, y, z = df[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].mean()
    transformer = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)
    lon, lat, alt = transformer.transform(x, y, z)
    g = normal_gravity(np.deg2rad(lat), alt)
    return float(g), float(lat), float(lon), float(alt)


def compute_gravity_from_gnss(gnss_file: Path) -> tuple[float, float, float, float]:
    """Return gravity magnitude [m/s^2] using the GNSS CSV."""
    df = pd.read_csv(gnss_file)
    x = df["X_ECEF_m"].mean()
    y = df["Y_ECEF_m"].mean()
    z = df["Z_ECEF_m"].mean()
    transformer = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)
    lon, lat, alt = transformer.transform(x, y, z)
    g = normal_gravity(np.deg2rad(lat), alt)
    return float(g), float(lat), float(lon), float(alt)


# ---------------------------------------------------------------------------
# Main CLI
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--mode",
        choices=["fusion", "truth"],
        default="fusion",
        help="validation mode",
    )
    p.add_argument(
        "--state-file",
        type=Path,
        default=Path("STATE_X001.txt"),
        help="ground truth trajectory file",
    )
    p.add_argument(
        "--gnss-file",
        type=Path,
        default=Path("GNSS_X002.csv"),
        help="GNSS CSV used to compute gravity for case 'gnss'",
    )
    p.add_argument(
        "--cases",
        nargs="+",
        choices=["gnss", "state"],
        default=["gnss", "state"],
        help="gravity cases to plot",
    )
    p.add_argument(
        "--results-dir",
        type=Path,
        default=Path("results"),
        help="directory containing *_kf_output.mat",
    )
    args = p.parse_args(argv)

    cases: list[tuple[str, float]] = []
    if "gnss" in args.cases:
        g_gnss, lat_gnss, lon_gnss, alt_gnss = compute_gravity_from_gnss(args.gnss_file)
        print(
            f"GNSS gravity: {g_gnss:.6f} m/s^2 at lat={lat_gnss:.6f}°, lon={lon_gnss:.6f}°, alt={alt_gnss:.2f} m"
        )
        cases.append(("gnss", g_gnss))
    if "state" in args.cases:
        g_state, lat_state, lon_state, alt_state = compute_gravity_from_state(args.state_file)
        print(
            f"STATE gravity: {g_state:.6f} m/s^2 at lat={lat_state:.6f}°, lon={lon_state:.6f}°, alt={alt_state:.2f} m"
        )
        cases.append(("state", g_state))

    df_state = pd.read_csv(
        args.state_file,
        sep="\s+",
        comment="#",
        names=[
            "count",
            "time",
            "X",
            "Y",
            "Z",
            "VX",
            "VY",
            "VZ",
            "q0",
            "q1",
            "q2",
            "q3",
        ],
    )
    truth_xyz = df_state[["X", "Y", "Z"]].to_numpy()

    methods = ["TRIAD", "SVD", "Davenport"]
    axes = ["X", "Y", "Z"]
    args.results_dir.mkdir(exist_ok=True)

    for method in methods:
        est_file = args.results_dir / f"IMU_X002_GNSS_X002_{method}_kf_output.mat"
        if args.mode == "fusion":
            if not est_file.exists():
                print(f"Estimator result '{est_file}' not found, skipping.")
                continue
            est = sio.loadmat(est_file)
            est_xyz = est.get("pos_ecef_m")
            if est_xyz is None:
                print(f"{est_file} missing 'pos_ecef_m', skipping.")
                continue

            for case, g in cases:
                for i, ax in enumerate(axes):
                    plt.figure(figsize=(10, 5))
                    plt.plot(truth_xyz[:, i], label="Truth")
                    plt.plot(est_xyz[:, i].squeeze(), label=f"{method}")
                    plt.xlabel("Sample")
                    plt.ylabel(f"{ax} [m]")
                    plt.title(f"{method} {ax}-axis ({case} g={g:.5f} m/s^2)")
                    plt.legend()
                    plt.grid(True)
                    fname = f"{method}_ECEF_{ax}_{case}.pdf"
                    plt.savefig(args.results_dir / fname)
                    plt.close()

                z_rmse = np.sqrt(np.mean((est_xyz[:, 2].squeeze() - truth_xyz[:, 2]) ** 2))
                print(f"{method} ({case}): Z-axis RMSE {z_rmse:.2f} m")
        else:  # truth mode
            for i, ax in enumerate(axes):
                plt.figure(figsize=(10, 5))
                plt.plot(truth_xyz[:, i], label="Truth")
                plt.xlabel("Sample")
                plt.ylabel(f"{ax} [m]")
                plt.title(f"STATE {ax}-axis")
                plt.legend()
                plt.grid(True)
                plt.savefig(args.results_dir / f"STATE_ECEF_{ax}_Truth.pdf")
                plt.close()


if __name__ == "__main__":  # pragma: no cover
    main()
