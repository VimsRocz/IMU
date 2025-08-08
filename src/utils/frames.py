"""Frame rotation helpers.

Provides tested ECEF↔NED rotation matrices, matching MATLAB's
``ecef_ned_rot`` utility.
"""
from __future__ import annotations

import numpy as np


def ecef_to_ned(lat_rad: float, lon_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """Return rotation matrices between ECEF and NED frames.

    Parameters
    ----------
    lat_rad, lon_rad : float
        Geodetic latitude and longitude in radians.

    Returns
    -------
    R_en, R_ne : ndarray
        ``R_en`` maps ECEF vectors to NED; ``R_ne`` is its transpose.
    """

    sL = np.sin(lat_rad)
    cL = np.cos(lat_rad)
    sLam = np.sin(lon_rad)
    cLam = np.cos(lon_rad)

    R_en = np.array([
        [-sL * cLam, -sL * sLam, cL],
        [-sLam, cLam, 0.0],
        [-cL * cLam, -cL * sLam, -sL],
    ])
    return R_en, R_en.T

