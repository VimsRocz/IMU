from __future__ import annotations

import json
import uuid
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.io as pio

from utils import ecef_to_geodetic


def ensure_deg_latlon(lat_in, lon_in):
    lat = float(lat_in)
    lon = float(lon_in)
    if abs(lat) <= np.pi + 1e-9 and abs(lon) <= 2 * np.pi + 1e-9:
        lat = np.degrees(lat)
        lon = np.degrees(lon)
    if lon > 180:
        lon = ((lon + 180) % 360) - 180
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError(f"Bad lat/lon after conversion: lat={lat}, lon={lon}")
    return lat, lon


def task1_reference_vectors(gnss_data: pd.DataFrame, output_dir: str | Path, run_id: str) -> Path:
    """Save a static world map showing the initial GNSS location.

    Parameters
    ----------
    gnss_data : pandas.DataFrame
        GNSS data containing either ``Latitude_deg``/``Longitude_deg`` or
        ECEF coordinates ``X_ECEF_m``/``Y_ECEF_m``/``Z_ECEF_m``.
    output_dir : str or Path
        Directory where the map and JSON metadata will be saved.
    run_id : str
        Identifier used to name the output files.

    Returns
    -------
    Path
        Path to the written PNG image.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if "Height_deg" in gnss_data.columns and "Height_m" not in gnss_data.columns:
        gnss_data = gnss_data.rename(columns={"Height_deg": "Height_m"})

    if {"Latitude_deg", "Longitude_deg"}.issubset(gnss_data.columns):
        # Some of the provided GNSS files contain zero-filled latitude/longitude
        # columns.  Treat rows where both coordinates are zero as invalid so
        # that we fall back to the ECEF-derived location instead.
        valid = (
            gnss_data["Latitude_deg"].notna()
            & gnss_data["Longitude_deg"].notna()
            & (
                (gnss_data["Latitude_deg"].abs() > 1e-9)
                | (gnss_data["Longitude_deg"].abs() > 1e-9)
            )
        )
        if valid.any():
            row = gnss_data.loc[valid].iloc[0]
            lat_raw = float(row["Latitude_deg"])
            lon_raw = float(row["Longitude_deg"])
        else:
            row = gnss_data.loc[
                (gnss_data["X_ECEF_m"] != 0)
                | (gnss_data["Y_ECEF_m"] != 0)
                | (gnss_data["Z_ECEF_m"] != 0)
            ].iloc[0]
            lat_raw, lon_raw, _ = ecef_to_geodetic(
                row["X_ECEF_m"], row["Y_ECEF_m"], row["Z_ECEF_m"]
            )
    else:
        row = gnss_data.loc[
            (gnss_data["X_ECEF_m"] != 0)
            | (gnss_data["Y_ECEF_m"] != 0)
            | (gnss_data["Z_ECEF_m"] != 0)
        ].iloc[0]
        lat_raw, lon_raw, _ = ecef_to_geodetic(
            row["X_ECEF_m"], row["Y_ECEF_m"], row["Z_ECEF_m"]
        )

    lat_deg, lon_deg = ensure_deg_latlon(lat_raw, lon_raw)

    fig = go.Figure()
    fig.add_scattergeo(
        lon=[lon_deg], lat=[lat_deg], mode="markers", marker=dict(size=10, color="red")
    )
    fig.update_layout(
        title="Task 1 — Initial GNSS location",
        geo=dict(
            projection_type="equirectangular",
            showcountries=True,
            showcoastlines=True,
            showland=True,
            lataxis=dict(showgrid=True, dtick=15),
            lonaxis=dict(showgrid=True, dtick=30),
        ),
    )

    png_path = output_dir / f"{run_id}_task1_location_map.png"
    pio.write_image(fig, png_path, width=1200, height=800, scale=2)
    print(f"[SAVE] {png_path}")

    info = {
        "plot_id": uuid.uuid4().hex,
        "latitude": lat_deg,
        "longitude": lon_deg,
    }
    info_path = output_dir / f"{run_id}_task1_location_map_info.json"
    with info_path.open("w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)

    return png_path
