"""INS/GNSS fusion debugging helper.

This script loads fused estimator outputs and a reference trajectory,
then runs a series of sanity checks.  It is intended to track down
large position or velocity errors by verifying frame alignment,
time synchronisation and IMU bias correction.

Usage
-----
python src/debug_fusion_pipeline.py --fused-file results/my_kf_output.npz \
    --truth-file STATE_X001.txt [--imu-file IMU_X001.dat]

All generated figures are saved in ``results/``.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple

import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import interp1d

from src.plot_fused_vs_truth import plot_fused_vs_truth


def load_estimate(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load estimator results from ``path``.

    Supports ``.npz`` files with ``time_s``, ``pos_ecef_m`` and ``vel_ecef_ms``
    arrays.  Additional formats can be added as needed.
    """
    data = np.load(path)
    t = data["time_s"].astype(float)
    pos = data["pos_ecef_m"].astype(float)
    vel = data["vel_ecef_ms"].astype(float)
    return t, pos, vel


def load_truth(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load reference trajectory from ``path``.

    The file must contain columns ``time_s``, ``X_ECEF_m``, ``Y_ECEF_m`` and
    ``Z_ECEF_m``.  Additional velocity columns are optional and loaded when
    present.
    """
    txt = np.loadtxt(path, delimiter=",", skiprows=1)
    t = txt[:, 0]
    pos = txt[:, 1:4]
    vel = txt[:, 4:7] if txt.shape[1] >= 7 else None
    return t, pos, vel


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fused-file", type=Path, required=True)
    parser.add_argument("--truth-file", type=Path, required=True)
    parser.add_argument("--imu-file", type=Path)
    parser.add_argument("--output", type=Path, default=Path("results"))
    args = parser.parse_args(argv)

    args.output.mkdir(parents=True, exist_ok=True)

    t_fused, pos_fused, vel_fused = load_estimate(args.fused_file)
    t_truth, pos_truth, vel_truth = load_truth(args.truth_file)

    print("Initial fused position:", pos_fused[0])
    print("Initial truth position:", pos_truth[0])
    print("Initial difference:", pos_fused[0] - pos_truth[0])

    plt.figure()
    plt.plot(t_fused, label="Fused time")
    plt.plot(t_truth, label="Truth time")
    plt.legend()
    plt.title("Timestamps: Fused vs Truth")
    plt.grid(True)
    plt.tight_layout()
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig1, str(args.output / "debug_timestamps"))
    plt.close()

    interp_truth_pos = np.vstack([
        interp1d(t_truth, pos_truth[:, i], fill_value="extrapolate")(t_fused)
        for i in range(3)
    ]).T
    error_pos = pos_fused - interp_truth_pos

    plt.figure()
    plt.plot(t_fused, np.linalg.norm(error_pos, axis=1))
    plt.xlabel("Time [s]")
    plt.ylabel("Position error [m]")
    plt.title("Position Error Over Time")
    plt.grid(True)
    plt.tight_layout()
    save_matlab_fig(fig2, str(args.output / "debug_error_norm"))
    plt.close()

    plot_fused_vs_truth(
        t_fused,
        interp_truth_pos,
        pos_fused,
        args.output / "debug_overlay_ecef.pdf",
        frame_label="ECEF",
    )

    final_error = np.linalg.norm(error_pos[-1])
    rmse = np.sqrt(np.mean(np.sum(error_pos ** 2, axis=1)))
    print(f"Final position error: {final_error:.3f} m")
    print(f"Position RMSE: {rmse:.3f} m")


if __name__ == "__main__":  # pragma: no cover
    main()
