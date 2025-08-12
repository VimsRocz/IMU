from __future__ import annotations

import numpy as np
from typing import Iterable, Tuple

from .ecef_llh import lla_to_ecef


def _rotation_ecef_to_ned(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    return np.array([
        [-np.sin(lat) * np.cos(lon), -np.sin(lon), -np.cos(lat) * np.cos(lon)],
        [-np.sin(lat) * np.sin(lon),  np.cos(lon), -np.cos(lat) * np.sin(lon)],
        [ np.cos(lat),               0.0,         -np.sin(lat)],
    ])


def ecef_to_ned(x: Iterable[float], y: Iterable[float], z: Iterable[float],
                lat0: float, lon0: float, h0: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert ECEF coordinates to local NED frame relative to reference."""
    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    x0, y0, z0 = lla_to_ecef(lat0, lon0, h0)
    R = _rotation_ecef_to_ned(lat0, lon0)
    diff = np.vstack((x - x0, y - y0, z - z0))
    ned = R @ diff
    return ned[0], ned[1], ned[2]


def ned_to_ecef(n: Iterable[float], e: Iterable[float], d: Iterable[float],
                lat0: float, lon0: float, h0: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert NED coordinates to ECEF."""
    n = np.asarray(n)
    e = np.asarray(e)
    d = np.asarray(d)
    x0, y0, z0 = lla_to_ecef(lat0, lon0, h0)
    R = _rotation_ecef_to_ned(lat0, lon0)
    ecef = np.linalg.inv(R) @ np.vstack((n, e, d))
    return ecef[0] + x0, ecef[1] + y0, ecef[2] + z0
