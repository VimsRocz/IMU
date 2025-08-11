"""Simple nearest city lookup used in Task 1 plotting."""
from __future__ import annotations
from typing import Tuple
import numpy as np

_CITIES = [
    ("Sydney", -33.8688, 151.2093),
    ("Melbourne", -37.8136, 144.9631),
    ("Perth", -31.9505, 115.8605),
    ("Adelaide", -34.9285, 138.6007),
    ("Brisbane", -27.4698, 153.0251),
    ("Darwin", -12.4634, 130.8456),
    ("Hobart", -42.8821, 147.3272),
    ("Canberra", -35.2809, 149.1300),
    ("London", 51.5074, -0.1278),
    ("New York", 40.7128, -74.0060),
    ("Tokyo", 35.6895, 139.6917),
    ("Singapore", 1.3521, 103.8198),
]
_R = 6371.0  # km


def _haversine(lat1, lon1, lat2, lon2):
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2) ** 2
    return 2 * _R * np.arcsin(np.sqrt(a))


def nearest_city(lat_deg: float, lon_deg: float) -> Tuple[str, float]:
    """Return nearest city name and distance in kilometres."""
    dists = [_haversine(lat_deg, lon_deg, c[1], c[2]) for c in _CITIES]
    i = int(np.argmin(dists))
    return _CITIES[i][0], float(dists[i])
