from __future__ import annotations
import os, math, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Optional deps
try:
    import cartopy.crs as ccrs
    import cartopy.feature as cfeature
    _HAS_CARTOPY = True
except Exception:
    _HAS_CARTOPY = False

# optional reverse geocode (best-effort, offline safe fallback)
try:
    import geopy
    from geopy.geocoders import Nominatim
    _HAS_GEOPY = True
except Exception:
    _HAS_GEOPY = False


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


def _ecef_to_geodetic_wgs84(x, y, z):
    """Convert ECEF [m] -> (lat, lon, h). Returns lat, lon in radians, h in meters.
    Robust closed-form Bowring method (WGS-84)."""
    a = 6378137.0
    f = 1.0/298.257223563
    e2 = f*(2-f)
    b = a*(1-f)
    ep2 = (a*a - b*b)/(b*b)

    lon = math.atan2(y, x)
    r = math.hypot(x, y)
    # init
    u = math.atan2(z*a, r*b)
    su, cu = math.sin(u), math.cos(u)
    lat = math.atan2(z + ep2*b*su*su*su, r - e2*a*cu*cu*cu)
    # one Newton step is usually enough
    for _ in range(2):
        s = math.sin(lat); c = math.cos(lat)
        N = a / math.sqrt(1 - e2*s*s)
        h = r/c - N
        lat = math.atan2(z + e2*N*s, r)

    s = math.sin(lat)
    N = a / math.sqrt(1 - e2*s*s)
    h = r/math.cos(lat) - N
    return lat, lon, h


def _gnss_to_latlon_deg(gnss_file: str) -> np.ndarray:
    """
    Return Nx2 array of [lat_deg, lon_deg] for rows with position.
    Accepts csv with columns:
      X_ECEF_m, Y_ECEF_m, Z_ECEF_m  (required if no Latitude_deg/Longitude_deg)
      or Latitude_deg, Longitude_deg (direct)
    """
    import csv
    latlon = []
    with open(gnss_file, newline="") as f:
        reader = csv.DictReader(f)
        has_ll = ("Latitude_deg" in reader.fieldnames) and ("Longitude_deg" in reader.fieldnames)
        for row in reader:
            if has_ll:
                try:
                    lat = float(row["Latitude_deg"])
                    lon = float(row["Longitude_deg"])
                    # Ignore obviously invalid zero-filled coordinates
                    if (
                        np.isfinite(lat)
                        and np.isfinite(lon)
                        and (abs(lat) > 1e-9 or abs(lon) > 1e-9)
                    ):
                        latlon.append([lat, lon])
                except Exception:
                    continue
            else:
                try:
                    x = float(row["X_ECEF_m"]); y = float(row["Y_ECEF_m"]); z = float(row["Z_ECEF_m"])
                    lat, lon, _h = _ecef_to_geodetic_wgs84(x, y, z)
                    latlon.append([math.degrees(lat), math.degrees(lon)])
                except Exception:
                    continue
    if not latlon:
        raise RuntimeError("No valid GNSS positions found (need either lat/lon or ECEF columns).")
    return np.array(latlon, dtype=float)


def _pretty_lat(lat):
    hemi = "N" if lat >= 0 else "S"
    return f"{abs(lat):.5f}°{hemi}"

def _pretty_lon(lon):
    hemi = "E" if lon >= 0 else "W"
    return f"{abs(lon):.5f}°{hemi}"


def _reverse_geocode(lat_deg: float, lon_deg: float, timeout_s: int = 4) -> str:
    """Best-effort; returns 'Unknown' on any failure or no network."""
    if not _HAS_GEOPY:
        return "Unknown"
    try:
        geolocator = Nominatim(user_agent="imu_task1", timeout=timeout_s)
        loc = geolocator.reverse((lat_deg, lon_deg), language="en", zoom=5)
        if loc and loc.address:
            # keep it short: City, Country (best effort)
            parts = [p.strip() for p in loc.address.split(",")]
            if len(parts) >= 2:
                return f"{parts[-3] if len(parts)>=3 else parts[-2]}, {parts[-1]}"
            return parts[-1]
    except Exception:
        pass
    return "Unknown"


def save_task1_worldmap_png(gnss_file: str, run_id: str, out_dir="PYTHON/results") -> str:
    """Create one rectangular world map PNG with colored Earth, GNSS point, labels."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / f"{run_id}_task1_location_map.png"

    # 1) Get lat/lon sequence, compute representative location
    latlon = _gnss_to_latlon_deg(gnss_file)
    latlon = np.apply_along_axis(lambda r: ensure_deg_latlon(r[0], r[1]), 1, latlon)
    lat_med = float(np.nanmedian(latlon[:, 0]))
    lon_med = float(np.nanmedian(latlon[:, 1]))
    lat_med, lon_med = ensure_deg_latlon(lat_med, lon_med)
    place = _reverse_geocode(lat_med, lon_med)

    # 2) Build figure
    figsize = (11, 6)  # wide rectangular
    if _HAS_CARTOPY:
        proj = ccrs.PlateCarree()
        fig = plt.figure(figsize=figsize, dpi=150)
        ax = plt.axes(projection=proj)
        ax.set_global()
        # colored earth (offline features)
        ax.add_feature(cfeature.OCEAN, facecolor="#9ecae1")   # light blue
        ax.add_feature(cfeature.LAND,  facecolor="#a1d99b")   # light green
        ax.add_feature(cfeature.COASTLINE, linewidth=0.5, edgecolor="0.3")
        ax.gridlines(draw_labels=True, linestyle="--", alpha=0.4, xlocs=np.arange(-180,181,30), ylocs=np.arange(-90,91,15))
        # GNSS track (thin), representative point (big)
        ax.plot(latlon[:,1], latlon[:,0], transform=proj, lw=0.6, alpha=0.6, color="k")
        ax.plot([lon_med], [lat_med], marker="o", markersize=6, color="crimson", transform=proj, zorder=5)
        # Text box with labels
        txt = f"Dataset: {run_id}\nLat: {_pretty_lat(lat_med)}  Lon: {_pretty_lon(lon_med)}\nPlace: {place}"
        ax.text(lon_med+3, lat_med, txt, transform=proj, fontsize=9, va="center", ha="left",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.3", alpha=0.9))
        ax.set_title("Task 1 — World Map (Plate Carrée)")

    else:
        # Fallback: plain rectangular axes with colored background & grid
        fig, ax = plt.subplots(figsize=figsize, dpi=150)
        ax.set_xlim(-180, 180); ax.set_ylim(-90, 90)
        ax.set_facecolor("#9ecae1")  # ocean
        # fake land tint (very rough): just a light overlay rectangle
        ax.add_patch(plt.Rectangle((-180,-90), 360, 180, facecolor="#a1d99b", alpha=0.35, zorder=0))
        # grid
        ax.set_xticks(np.arange(-180, 181, 30)); ax.set_yticks(np.arange(-90, 91, 15))
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_xlabel("Longitude [deg]"); ax.set_ylabel("Latitude [deg]")
        # track & point
        ax.plot(latlon[:,1], latlon[:,0], lw=0.6, alpha=0.7, color="k")
        ax.plot([lon_med], [lat_med], marker="o", markersize=6, color="crimson")
        txt = f"Dataset: {run_id}\nLat: {_pretty_lat(lat_med)}  Lon: {_pretty_lon(lon_med)}\nPlace: {place}"
        ax.text(lon_med+3, lat_med, txt, fontsize=9, va="center", ha="left",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.3", alpha=0.9))
        ax.set_title("Task 1 — World Map (rectangular)")

    # 3) Save single PNG
    fig.tight_layout()
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(out_png.with_suffix('')))
    plt.close(fig)
    print(f"[Task1] Saved location map -> {out_png}")
    return str(out_png)
