from __future__ import annotations

import json
import uuid
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go
import plotly.io as pio

from utils import ecef_to_geodetic


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

    if {"Latitude_deg", "Longitude_deg"}.issubset(gnss_data.columns):
        lat = float(gnss_data.loc[gnss_data["Latitude_deg"].notna()].iloc[0]["Latitude_deg"])
        lon = float(gnss_data.loc[gnss_data["Longitude_deg"].notna()].iloc[0]["Longitude_deg"])
    else:
        row = gnss_data.loc[
            (gnss_data["X_ECEF_m"] != 0)
            | (gnss_data["Y_ECEF_m"] != 0)
            | (gnss_data["Z_ECEF_m"] != 0)
        ].iloc[0]
        lat, lon, _ = ecef_to_geodetic(row["X_ECEF_m"], row["Y_ECEF_m"], row["Z_ECEF_m"])

    fig = go.Figure(
        go.Scattergeo(lat=[lat], lon=[lon], mode="markers", marker=dict(size=10, color="red"))
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

    info = {
        "plot_id": uuid.uuid4().hex,
        "latitude": lat,
        "longitude": lon,
    }
    info_path = output_dir / f"{run_id}_task1_location_map_info.json"
    with info_path.open("w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)

    print(
        "Task 1: saved static map ->"
        f" {png_path} and info JSON"
    )

    return png_path
