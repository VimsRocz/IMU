"""Task 1: plot GNSS track on a whole-earth map.

This script loads GNSS latitude/longitude data using ``src.paths`` and
produces a global map using Cartopy.  The plot is saved as a single PNG
image.
"""

from __future__ import annotations

import argparse
from pathlib import Path, Path as _Path
import sys as _sys

# Make PYTHON/src importable regardless of CWD
_PY_SRC = _Path(__file__).resolve().parents[1] / "src"
if str(_PY_SRC) not in _sys.path:
    _sys.path.insert(0, str(_PY_SRC))
REPO_ROOT = _PY_SRC.parents[2]

import cartopy.crs as ccrs  # type: ignore
import matplotlib.pyplot as plt
import pandas as pd
from geopy.extra.rate_limiter import RateLimiter
from geopy.geocoders import Nominatim

from paths import gnss_path, PY_RES_DIR, ensure_py_results


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
    ax.stock_img()
    gl = ax.gridlines(draw_labels=True, linewidth=0.5, linestyle="--", color="gray")
    gl.top_labels = False
    gl.right_labels = False

    dataset_label = f"{Path(imu_file).stem}_{Path(gnss_file).stem}"
    lons = gnss["Longitude_deg"].to_numpy()
    lats = gnss["Latitude_deg"].to_numpy()
    ax.scatter(lons, lats, s=20, c="red", transform=ccrs.PlateCarree())

    # Annotate GNSS points with dataset name and coordinates
    for lon, lat in zip(lons, lats):
        ax.text(
            lon,
            lat,
            f"{dataset_label}\n{lat:.4f}, {lon:.4f}",
            fontsize=6,
            transform=ccrs.PlateCarree(),
            ha="left",
            va="bottom",
        )

    # Reverse geocode to add nearby place names
    geolocator = Nominatim(user_agent="imu_task1_plot")
    reverse = RateLimiter(
        geolocator.reverse, min_delay_seconds=1, swallow_exceptions=True
    )
    seen_places: set[str] = set()
    for lon, lat in zip(lons, lats):
        location = reverse((lat, lon), language="en", exactly_one=True)
        if not location or not getattr(location, "raw", None):
            continue
        addr = location.raw.get("address", {})
        place = addr.get("city") or addr.get("town") or addr.get("village") or addr.get("state")
        if place and place not in seen_places:
            seen_places.add(place)
            ax.text(
                lon,
                lat,
                place,
                fontsize=8,
                color="blue",
                transform=ccrs.PlateCarree(),
                ha="right",
                va="top",
            )

    tag = f"{dataset_label}_{method}_task1_location_map"
    png_path = PY_RES_DIR / f"{tag}.png"
    fig.savefig(png_path, dpi=300)
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
