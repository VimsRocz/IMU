#!/usr/bin/env python3
"""Generate a PNG map for Task 1 results from saved artifacts.

The script reads a ``*_task1_artifacts.npz`` file and the associated JSON
results file, then plots the initial location on a whole-earth map with
annotations for dataset name, place name, and latitude/longitude values.

Example
-------
python plot_task1_results.py --artifacts IMU_X002_GNSS_X002_TRIAD_task1_artifacts.npz \
    --json IMU_X002_GNSS_X002_TRIAD_task1_results.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cartopy.crs as ccrs  # type: ignore
import matplotlib.pyplot as plt
import numpy as np
from geopy.extra.rate_limiter import RateLimiter
from geopy.geocoders import Nominatim


def plot_from_artifacts(npz_path: Path, json_path: Path) -> Path:
    """Create a world map from Task 1 artifacts and save as PNG.

    Parameters
    ----------
    npz_path : Path
        Path to ``*_task1_artifacts.npz`` containing ``lat0_deg`` and ``lon0_deg``.
    json_path : Path
        Path to the accompanying JSON results file.

    Returns
    -------
    Path
        Location of the written PNG file.
    """
    data = np.load(npz_path)
    lat = float(data["lat0_deg"])
    lon = float(data["lon0_deg"])

    meta = json.loads(json_path.read_text()).get("meta", {})
    dataset_name = meta.get("dataset_id", npz_path.stem.replace("_task1_artifacts", ""))

    # Reverse geocode to obtain a human-readable place name.
    geolocator = Nominatim(user_agent="imu_task1_plot")
    try:
        location = geolocator.reverse((lat, lon), language="en", exactly_one=True, timeout=10)
    except Exception:
        location = None
    place = ""
    if location and getattr(location, "raw", None):
        addr = location.raw.get("address", {})
        place = (
            addr.get("city")
            or addr.get("town")
            or addr.get("village")
            or addr.get("state")
            or ""
        )

    fig = plt.figure(figsize=(10, 5))
    ax = plt.axes(projection=ccrs.PlateCarree())
    ax.set_global()
    ax.stock_img()
    gl = ax.gridlines(draw_labels=True, linewidth=0.5, linestyle="--", color="gray")
    gl.top_labels = False
    gl.right_labels = False

    ax.scatter([lon], [lat], color="red", transform=ccrs.PlateCarree())
    label = f"{dataset_name}\n{place}\nlat: {lat:.4f}\nlon: {lon:.4f}"
    ax.text(lon + 3, lat + 3, label, fontsize=8, transform=ccrs.PlateCarree())

    out_path = npz_path.with_name(npz_path.name.replace("_artifacts.npz", "_location_map.png"))
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Task 1 artifacts to PNG")
    parser.add_argument("--artifacts", required=True, help="Path to *_task1_artifacts.npz")
    parser.add_argument("--json", required=True, help="Path to *_task1_results.json")
    args = parser.parse_args()

    npz_path = Path(args.artifacts)
    json_path = Path(args.json)
    png_path = plot_from_artifacts(npz_path, json_path)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
