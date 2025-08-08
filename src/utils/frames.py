"""Reference frame rotation utilities.

Canonical ECEF↔NED rotation matrices shared across the codebase and
mirroring the MATLAB implementation.

Functions
---------
R_ecef_to_ned(lat_rad, lon_rad)
    Rotation matrix from ECEF to NED.
R_ned_to_ecef(lat_rad, lon_rad)
    Rotation matrix from NED to ECEF (transpose of ``R_ecef_to_ned``).
ecef_vec_to_ned(vec_ecef, lat_rad, lon_rad)
    Rotate one or more ECEF vectors to NED.
ned_vec_to_ecef(vec_ned, lat_rad, lon_rad)
    Rotate one or more NED vectors to ECEF.

The matrices are orthonormal and follow the right-handed NED convention.
"""
from __future__ import annotations

import numpy as np


def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotation matrix from ECEF to NED.

    Parameters
    ----------
    lat_rad, lon_rad : float
        Geodetic latitude and longitude in **radians**.

    Returns
    -------
    ndarray of shape (3, 3)
        Matrix that maps ECEF vectors into the local NED frame. Rows are
        ordered [North, East, Down] expressed in ECEF coordinates.
    """
    sphi, cphi = np.sin(lat_rad), np.cos(lat_rad)
    slam, clam = np.sin(lon_rad), np.cos(lon_rad)
    R = np.array([
        [-sphi * clam, -sphi * slam,  cphi],
        [      -slam,        clam,  0.0],
        [-cphi * clam, -cphi * slam, -sphi],
    ])
    return R


def R_ned_to_ecef(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotation matrix from NED to ECEF."""
    return R_ecef_to_ned(lat_rad, lon_rad).T


def ecef_vec_to_ned(vec_ecef: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotate ECEF vector(s) to NED.

    Parameters
    ----------
    vec_ecef : array_like, shape (..., 3)
        Vector(s) expressed in ECEF coordinates.
    lat_rad, lon_rad : float
        Geodetic latitude and longitude in radians.

    Returns
    -------
    ndarray
        Vector(s) expressed in NED coordinates with the same leading shape
        as ``vec_ecef``.
    """
    R = R_ecef_to_ned(lat_rad, lon_rad)
    vec = np.asarray(vec_ecef)
    return (R @ vec.T).T


def ned_vec_to_ecef(vec_ned: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotate NED vector(s) to ECEF."""
    R = R_ned_to_ecef(lat_rad, lon_rad)
    vec = np.asarray(vec_ned)
    return (R @ vec.T).T
