"""Generate example Task 4–6 plots using synthetic data.

The script serves as a thin wrapper around :func:`plot_task_grid` and prints
sanity information before saving the figures under ``PYTHON/results``.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

# make src importable
sys.path.append(str(Path(__file__).resolve().parents[1] / "src"))
from task_plot_grid import plot_task_grid


DATASET = "IMU_X002_GNSS_X002_TRIAD"
OUT_DIR = Path("PYTHON/results")


def _synth(n: int = 500):
    t = np.linspace(0.0, 10.0, n)
    pos = np.column_stack((np.sin(t), np.cos(t), np.sin(0.5 * t)))
    vel = np.gradient(pos, t, axis=0)
    acc = np.gradient(vel, t, axis=0)
    return {"pos": pos, "vel": vel, "acc": acc}, t


def main() -> None:
    fused, t = _synth()
    gnss = {k: v + 0.05 * np.random.randn(*v.shape) for k, v in fused.items()}
    imu = {k: v + 0.1 * np.random.randn(*v.shape) for k, v in fused.items()}
    truth = {k: v + 0.02 * np.random.randn(*v.shape) for k, v in fused.items()}

    for frame in ["NED", "ECEF", "BODY"]:
        plot_task_grid(
            task=4,
            frame=frame,
            dataset=DATASET,
            out_dir=OUT_DIR,
            t_fused=t,
            fused=fused,
            t_gnss=t,
            gnss=gnss,
            t_imu=t,
            imu=imu,
        )
        plot_task_grid(
            task=5,
            frame=frame,
            dataset=DATASET,
            out_dir=OUT_DIR,
            t_fused=t,
            fused=fused,
        )
        plot_task_grid(
            task=6,
            frame=frame,
            dataset=DATASET,
            out_dir=OUT_DIR,
            t_fused=t,
            fused=fused,
            t_truth=t,
            truth=truth,
        )


if __name__ == "__main__":
    main()
