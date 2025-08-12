from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from src.utils.plot_io import save_plot, ensure_output_dirs
from src.utils.ecef_llh import ecef_to_lla
from src.utils.io_paths import infer_run_name


CITIES = {
    "Hamburg": (53.5511, 9.9937),
    "Bremen": (53.0793, 8.8017),
}


def nearest_city(lat: float, lon: float) -> str | None:
    min_dist = float("inf")
    nearest = None
    for name, (clat, clon) in CITIES.items():
        d = (lat - clat) ** 2 + (lon - clon) ** 2
        if d < min_dist:
            min_dist = d
            nearest = name
    return nearest


def run(gnss_file: Path, imu_file: Path, run_name: str | None = None) -> None:
    if run_name is None:
        run_name = infer_run_name(gnss_file, imu_file)

    data = np.genfromtxt(gnss_file, delimiter=",", names=True)
    x, y, z = data["X_ECEF_m"], data["Y_ECEF_m"], data["Z_ECEF_m"]
    lat, lon, _ = ecef_to_lla(x, y, z)

    out_dir = ensure_output_dirs(run_name, "Task_1")

    try:
        import cartopy.crs as ccrs  # type: ignore
        proj = ccrs.PlateCarree()
        fig = plt.figure(figsize=(10, 5))
        ax = plt.axes(projection=proj)
        ax.coastlines()
        ax.gridlines(draw_labels=True)
        ax.plot(lon, lat, transform=ccrs.Geodetic(), label="GNSS track")
        ax.legend()
        ax.plot(lon[0], lat[0], "go", label="start")
        ax.plot(lon[-1], lat[-1], "ro", label="end")
        start_city = nearest_city(lat[0], lon[0])
        end_city = nearest_city(lat[-1], lon[-1])
        if start_city:
            ax.annotate(start_city, (lon[0], lat[0]))
        if end_city:
            ax.annotate(end_city, (lon[-1], lat[-1]))
        save_plot(fig, out_dir, "task1_world_track")
    except Exception:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.set_xlim(-180, 180)
        ax.set_ylim(-90, 90)
        ax.set_xlabel("Longitude [deg]")
        ax.set_ylabel("Latitude [deg]")
        ax.plot(lon, lat, label="GNSS track")
        ax.legend()
        ax.plot(lon[0], lat[0], "go", label="start")
        ax.plot(lon[-1], lat[-1], "ro", label="end")
        start_city = nearest_city(lat[0], lon[0])
        end_city = nearest_city(lat[-1], lon[-1])
        if start_city:
            ax.annotate(start_city, (lon[0], lat[0]))
        if end_city:
            ax.annotate(end_city, (lon[-1], lat[-1]))
        save_plot(fig, out_dir, "task1_world_track")
    finally:
        plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Task 1: world map track")
    parser.add_argument("--gnss", type=Path, required=True, help="Path to GNSS CSV file")
    parser.add_argument("--imu", type=Path, required=True, help="Path to IMU data file")
    parser.add_argument("--run", type=str, default=None, help="Run name (e.g., X001)")
    args = parser.parse_args()
    run(args.gnss, args.imu, args.run)
