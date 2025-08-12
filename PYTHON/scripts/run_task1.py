"""Task 1: plot GNSS track on a whole-earth map.

This script loads GNSS latitude/longitude data using ``src.paths`` and
produces a global map using Cartopy.  The plot is saved both as a PNG image
and as a pickled Matplotlib figure for interactive reuse.
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import cartopy.crs as ccrs  # type: ignore
import matplotlib.pyplot as plt
import pandas as pd

from src.paths import gnss_path, imu_path, PY_RES_DIR, ensure_py_results


def run(imu_file: str, gnss_file: str, method: str = "TRIAD") -> None:
    """Generate a whole-earth location plot for *Task 1*.

    Parameters
    ----------
    imu_file : str
        IMU dataset filename (e.g. ``IMU_X002.dat``).
    gnss_file : str
        GNSS dataset filename (e.g. ``GNSS_X002.csv``).
    method : str, optional
        Name of the attitude initialisation method, by default ``"TRIAD"``.
    """

    ensure_py_results()

    # Load GNSS coordinates from standard DATA directory
    gnss = pd.read_csv(gnss_path(gnss_file))

    # Prepare whole-earth plot
    fig = plt.figure(figsize=(10, 5))
    ax = plt.axes(projection=ccrs.PlateCarree())
    ax.set_global()
    ax.coastlines()
    ax.gridlines(draw_labels=True)
    ax.scatter(
        gnss["Longitude_deg"],
        gnss["Latitude_deg"],
        s=10,
        c="red",
        transform=ccrs.PlateCarree(),
    )

    imu_name = Path(imu_file).stem
    gnss_name = Path(gnss_file).stem
    tag = f"{imu_name}_{gnss_name}_{method}_task1_location_map"

    # Save static image
    png_path = PY_RES_DIR / f"{tag}.png"
    fig.savefig(png_path, dpi=300)

    # Save interactive figure using pickle
    pkl_path = PY_RES_DIR / f"{tag}.pickle"
    with open(pkl_path, "wb") as fh:
        pickle.dump(fig, fh)

    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Task 1: whole-earth plot")
    parser.add_argument("--imu", required=True, help="IMU dataset filename")
    parser.add_argument("--gnss", required=True, help="GNSS dataset filename")
    parser.add_argument(
        "--method",
        default="TRIAD",
        help="Attitude initialisation method name",
    )
    args = parser.parse_args()
    run(args.imu, args.gnss, args.method)

