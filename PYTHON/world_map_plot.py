#!/usr/bin/env python3
"""
Plot sample geographic dataset on a rectangular world map using Cartopy.

This script displays latitude/longitude locations with annotations for the
place name and dataset name. The figure is saved as ``world_plot.png`` in the
current directory.
"""

import cartopy.crs as ccrs
import matplotlib.pyplot as plt


def plot_dataset(data):
    """Plot data points on a global PlateCarree map and save to a PNG."""
    fig = plt.figure(figsize=(12, 6))
    ax = plt.axes(projection=ccrs.PlateCarree())
    ax.set_global()
    ax.stock_img()
    gl = ax.gridlines(draw_labels=True, linewidth=0.5, color="gray", alpha=0.5)
    gl.top_labels = gl.right_labels = False

    for entry in data:
        lon = entry["lon"]
        lat = entry["lat"]
        dataset_name = entry["dataset"]
        place_name = entry["name"]
        ax.scatter(lon, lat, color="red", transform=ccrs.PlateCarree())
        label = f"{dataset_name}\n{place_name}\nlat: {lat}\nlon: {lon}"
        ax.text(lon + 3, lat + 3, label, fontsize=8, transform=ccrs.PlateCarree())

    ax.set_title("World dataset locations")
    plt.savefig("world_plot.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    sample_data = [
        {"dataset": "SampleDataset", "name": "New York", "lat": 40.7128, "lon": -74.0060},
        {"dataset": "SampleDataset", "name": "London", "lat": 51.5074, "lon": -0.1278},
        {"dataset": "SampleDataset", "name": "Tokyo", "lat": 35.6895, "lon": 139.6917},
    ]
    plot_dataset(sample_data)
