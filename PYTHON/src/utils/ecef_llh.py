from __future__ import annotations

from math import atan2, sqrt, sin, cos
from typing import Iterable, Tuple
import numpy as np

try:
    from pyproj import Transformer
    _TRANSFORMER = Transformer.from_crs("epsg:4978", "epsg:4326", always_xy=True)
except Exception:  # pragma: no cover - optional dependency
    _TRANSFORMER = None


_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3
_WGS84_B = _WGS84_A * sqrt(1 - _WGS84_E2)


def ecef_to_lla(x: Iterable[float], y: Iterable[float], z: Iterable[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert ECEF coordinates to latitude, longitude, altitude."""
    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    if _TRANSFORMER is not None:
        lon, lat, alt = _TRANSFORMER.transform(x, y, z)
        return np.array(lat), np.array(lon), np.array(alt)

    # Fallback algorithm based on WGS-84 ellipsoid
    p = np.sqrt(x ** 2 + y ** 2)
    th = np.arctan2(_WGS84_A * z, _WGS84_B * p)
    lon = np.arctan2(y, x)
    lat = np.arctan2(z + (np.sqrt(_WGS84_A**2 - _WGS84_B**2) ** 2 / _WGS84_B) * np.sin(th) ** 3,
                      p - _WGS84_E2 * _WGS84_A * np.cos(th) ** 3)
    N = _WGS84_A / np.sqrt(1 - _WGS84_E2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), alt


def lla_to_ecef(lat: Iterable[float], lon: Iterable[float], alt: Iterable[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert latitude, longitude, altitude to ECEF coordinates."""
    lat = np.radians(np.asarray(lat))
    lon = np.radians(np.asarray(lon))
    alt = np.asarray(alt)

    N = _WGS84_A / np.sqrt(1 - _WGS84_E2 * np.sin(lat) ** 2)
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - _WGS84_E2) + alt) * np.sin(lat)
    return x, y, z
