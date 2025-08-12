from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from src.utils.plot_io import save_plot, ensure_output_dirs
from src.utils.ecef_llh import ecef_to_lla
from src.utils.frames import ecef_to_ned
from src.utils.io_paths import infer_run_name


def run(gnss_file: Path, imu_file: Path, run_name: str | None = None) -> None:
    if run_name is None:
        run_name = infer_run_name(gnss_file, imu_file)

    gnss = np.genfromtxt(gnss_file, delimiter=",", names=True)
    t_gnss = gnss["Posix_Time"]
    x, y, z = gnss["X_ECEF_m"], gnss["Y_ECEF_m"], gnss["Z_ECEF_m"]
    vx, vy, vz = gnss["VX_ECEF_mps"], gnss["VY_ECEF_mps"], gnss["VZ_ECEF_mps"]
    ax = np.gradient(vx, t_gnss)
    ay = np.gradient(vy, t_gnss)
    az = np.gradient(vz, t_gnss)

    lat0, lon0, _ = ecef_to_lla(x[0], y[0], z[0])
    n, e, d = ecef_to_ned(x, y, z, lat0, lon0)
    vn, ve, vd = ecef_to_ned(vx, vy, vz, lat0, lon0)
    an, ae, ad = ecef_to_ned(ax, ay, az, lat0, lon0)

    imu = np.genfromtxt(imu_file, usecols=(1,5,6,7), invalid_raise=False)
    t_imu = imu[:, 0]
    acc_body = imu[:, 1:]

    out_dir = ensure_output_dirs(run_name, "Task_4")

    fig, axes = plt.subplots(3, 3, figsize=(15, 12), sharex="col")

    # Body frame
    axes[0, 0].axis("off")
    axes[0, 1].axis("off")
    axes[0, 2].plot(t_imu, acc_body[:, 0], label="Ax")
    axes[0, 2].plot(t_imu, acc_body[:, 1], label="Ay")
    axes[0, 2].plot(t_imu, acc_body[:, 2], label="Az")
    axes[0, 2].set_title("Body Accel [m/s²]")
    axes[0, 2].legend()

    # ECEF frame
    axes[1, 0].plot(t_gnss, x, label="X")
    axes[1, 0].plot(t_gnss, y, label="Y")
    axes[1, 0].plot(t_gnss, z, label="Z")
    axes[1, 0].set_title("ECEF Position [m]")
    axes[1, 0].legend()

    axes[1, 1].plot(t_gnss, vx, label="VX")
    axes[1, 1].plot(t_gnss, vy, label="VY")
    axes[1, 1].plot(t_gnss, vz, label="VZ")
    axes[1, 1].set_title("ECEF Velocity [m/s]")
    axes[1, 1].legend()

    axes[1, 2].plot(t_gnss, ax, label="AX")
    axes[1, 2].plot(t_gnss, ay, label="AY")
    axes[1, 2].plot(t_gnss, az, label="AZ")
    axes[1, 2].set_title("ECEF Accel [m/s²]")
    axes[1, 2].legend()

    # NED frame
    axes[2, 0].plot(t_gnss, n, label="N")
    axes[2, 0].plot(t_gnss, e, label="E")
    axes[2, 0].plot(t_gnss, d, label="D")
    axes[2, 0].set_title("NED Position [m]")
    axes[2, 0].legend()

    axes[2, 1].plot(t_gnss, vn, label="VN")
    axes[2, 1].plot(t_gnss, ve, label="VE")
    axes[2, 1].plot(t_gnss, vd, label="VD")
    axes[2, 1].set_title("NED Velocity [m/s]")
    axes[2, 1].legend()

    axes[2, 2].plot(t_gnss, an, label="AN")
    axes[2, 2].plot(t_gnss, ae, label="AE")
    axes[2, 2].plot(t_gnss, ad, label="AD")
    axes[2, 2].set_title("NED Accel [m/s²]")
    axes[2, 2].legend()

    for col in range(3):
        axes[2, col].set_xlabel("Time [s]")

    save_plot(fig, out_dir, "task4_frame_overlays")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Task 4: multi-frame overlays")
    parser.add_argument("--gnss", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--run", type=str, default=None)
    args = parser.parse_args()
    run(args.gnss, args.imu, args.run)
