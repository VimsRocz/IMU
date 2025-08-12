from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from src.utils.plot_io import save_plot, ensure_output_dirs
from src.utils.io_paths import infer_run_name


def run(gnss_file: Path, imu_file: Path, run_name: str | None = None) -> None:
    if run_name is None:
        run_name = infer_run_name(gnss_file, imu_file)

    state = np.genfromtxt(Path(gnss_file).resolve().parents[1] / "Truth" / f"STATE_{run_name}.txt",
                           usecols=(1,8,9,10,11), comments="#", invalid_raise=False)
    t = state[:, 0]
    state_q = state[:, 1:]

    # Placeholder measured quaternion: constant orientation
    meas_q = np.tile(state_q[0], (len(t), 1))

    out_dir = ensure_output_dirs(run_name, "Task_3")

    # Figure A: measured quaternion components
    figA, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    comps = ["q_w", "q_x", "q_y", "q_z"]
    for i in range(4):
        axes[i].plot(t, meas_q[:, i])
        axes[i].set_ylabel(comps[i])
    axes[-1].set_xlabel("Time [s]")
    figA.suptitle("Measured quaternion components")
    save_plot(figA, out_dir, "task3_measured_quat")
    plt.close(figA)

    # Figure B: state quaternion components
    figB, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    for i in range(4):
        axes[i].plot(t, state_q[:, i])
        axes[i].set_ylabel(comps[i])
    axes[-1].set_xlabel("Time [s]")
    figB.suptitle("STATE quaternion components")
    save_plot(figB, out_dir, "task3_state_quat")
    plt.close(figB)

    # Figure C: comparison
    figC, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    rmse = np.sqrt(np.mean((meas_q - state_q) ** 2, axis=0))
    for i in range(4):
        axes[i].plot(t, meas_q[:, i], label="measured")
        axes[i].plot(t, state_q[:, i], label="state")
        axes[i].set_ylabel(comps[i])
        axes[i].legend()
        axes[i].annotate(f"RMSE={rmse[i]:.3f}", xy=(0.05, 0.8), xycoords="axes fraction")
    axes[-1].set_xlabel("Time [s]")
    figC.suptitle("Quaternion comparison")
    save_plot(figC, out_dir, "task3_quat_comparison")
    plt.close(figC)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Task 3: quaternion comparison")
    parser.add_argument("--gnss", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--run", type=str, default=None)
    args = parser.parse_args()
    run(args.gnss, args.imu, args.run)
