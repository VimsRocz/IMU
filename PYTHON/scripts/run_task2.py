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

    gnss_time = np.genfromtxt(gnss_file, delimiter=",", names=True)["Posix_Time"]
    imu_time = np.genfromtxt(imu_file, usecols=(1,), invalid_raise=False)

    dt_gnss = np.diff(gnss_time)
    dt_imu = np.diff(imu_time)

    fs_gnss = 1.0 / dt_gnss.mean() if dt_gnss.size else float("nan")
    fs_imu = 1.0 / dt_imu.mean() if dt_imu.size else float("nan")

    out_dir = ensure_output_dirs(run_name, "Task_2")

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes[0].plot(gnss_time[1:], dt_gnss)
    axes[0].set_title("GNSS dt")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("dt [s]")

    axes[1].plot(imu_time[1:], dt_imu)
    axes[1].set_title("IMU dt")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("dt [s]")

    axes[2].axis("off")
    text = (f"fs_GNSS = {fs_gnss:.2f} Hz\n"
            f"fs_IMU = {fs_imu:.2f} Hz\n"
            f"dt_GNSS mean={dt_gnss.mean():.3f} s\n"
            f"dt_IMU mean={dt_imu.mean():.3f} s")
    axes[2].text(0.1, 0.5, text)

    save_plot(fig, out_dir, "task2_dt_fs")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Task 2: dt/fs plots")
    parser.add_argument("--gnss", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--run", type=str, default=None)
    args = parser.parse_args()
    run(args.gnss, args.imu, args.run)
