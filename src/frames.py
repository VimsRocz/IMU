import numpy as np


def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotation matrix from ECEF to NED.

    Parameters
    ----------
    lat_rad : float
        Geodetic latitude in **radians**.
    lon_rad : float
        Longitude in **radians**.

    Returns
    -------
    ndarray of shape (3, 3)
        Matrix ``R`` such that ``v_ned = R @ v_ecef``.
    """
    sphi, cphi = np.sin(lat_rad), np.cos(lat_rad)
    slam, clam = np.sin(lon_rad), np.cos(lon_rad)
    R = np.array([
        [-sphi * clam, -sphi * slam, cphi],
        [ -slam,        clam,        0.0],
        [-cphi * clam, -cphi * slam, -sphi],
    ])
    return R


def R_ned_to_ecef(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotation matrix from NED to ECEF."""
    return R_ecef_to_ned(lat_rad, lon_rad).T


def ecef_vec_to_ned(vec_ecef: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotate ECEF vectors into the NED frame.

    Parameters
    ----------
    vec_ecef : array_like (...,3)
        Vectors expressed in ECEF coordinates.
    lat_rad, lon_rad : float
        Site latitude and longitude in radians.
    """
    vec = np.asarray(vec_ecef)
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R @ vec.T).T


def ned_vec_to_ecef(vec_ned: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotate NED vectors into the ECEF frame."""
    vec = np.asarray(vec_ned)
    R = R_ned_to_ecef(lat_rad, lon_rad)
    return (R @ vec.T).T
