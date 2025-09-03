from __future__ import annotations

import numpy as np
from typing import Iterable, Tuple

from utils.ecef_llh import lla_to_ecef
from utils import compute_C_ECEF_to_NED


def _validate_ned_rotation(lat_rad: float, lon_rad: float, R: np.ndarray) -> None:
    """Check that the provided rotation maps gravity to positive down."""

    up = np.array([
        np.cos(lat_rad) * np.cos(lon_rad),
        np.cos(lat_rad) * np.sin(lon_rad),
        np.sin(lat_rad),
    ])
    down_ned = R @ (-up)
    if down_ned[2] <= 0:
        raise ValueError("NED frame expects positive down; rotation has wrong sign")


def _rotation_ecef_to_ned(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    R = compute_C_ECEF_to_NED(lat, lon)
    _validate_ned_rotation(lat, lon, R)
    return R


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
    # Ensure gravity direction is positive down
    if (R @ (-np.array([x0, y0, z0])))[2] <= 0:
        raise ValueError("NED conversion produced non-positive down component for gravity")
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


def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotation matrix from ECEF to NED.

    Parameters
    ----------
    lat_rad, lon_rad : float
        Geodetic latitude and longitude in **radians**.
    """
    R = compute_C_ECEF_to_NED(lat_rad, lon_rad)
    _validate_ned_rotation(lat_rad, lon_rad, R)
    return R


def ecef_vec_to_ned(
    vec_ecef: Iterable[float], lat_rad: float, lon_rad: float
) -> np.ndarray:
    """Rotate one or more ECEF vectors into the NED frame.

    Parameters
    ----------
    vec_ecef : array_like
        A single 3‑element vector or an array of shape ``(N, 3)`` containing
        multiple ECEF vectors.  The returned array preserves this shape.
    lat_rad, lon_rad : float
        Geodetic latitude and longitude in **radians**.

    Returns
    -------
    numpy.ndarray
        The input vector(s) rotated into the NED frame with the same leading
        dimensions as ``vec_ecef``.
    """

    vec = np.asarray(vec_ecef)
    return vec @ R_ecef_to_ned(lat_rad, lon_rad).T
