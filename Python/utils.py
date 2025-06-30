import numpy as np
from typing import Tuple, Optional
import pathlib
import subprocess
import sys


def ensure_dependencies(requirements: Optional[pathlib.Path] = None) -> None:
    """Install packages from ``requirements.txt`` if key deps are missing."""
    try:
        import tabulate  # noqa: F401
        import tqdm  # noqa: F401
    except ModuleNotFoundError:
        if requirements is None:
            requirements = pathlib.Path(__file__).resolve().parent / "requirements.txt"
        else:
            requirements = pathlib.Path(requirements)
        print("Installing Python dependencies ...")
        subprocess.check_call([
            sys.executable,
            "-m",
            "pip",
            "install",
            "-r",
            str(requirements),
        ])


def get_data_file(filename: str) -> pathlib.Path:
    """Return the full path to *filename* searching common data folders.

    The function mirrors the behaviour of the MATLAB ``get_data_file`` helper
    and allows scripts to be executed from arbitrary locations.  Searches the
    ``Data/`` folder next to this file, ``Python/data`` and the repository root
    for ``filename``.  ``FileNotFoundError`` is raised if the file cannot be
    located.
    """

    script_dir = pathlib.Path(__file__).resolve().parent
    search_dirs = [
        script_dir.parent / "Data",
        script_dir / "data",
        script_dir.parent,
    ]

    for d in search_dirs:
        candidate = d / filename
        if candidate.exists():
            return candidate

    raise FileNotFoundError(f"Data file not found: {filename}")


def detect_static_interval(accel_data, gyro_data, window_size=200,
                           accel_var_thresh=0.01, gyro_var_thresh=1e-6,
                           min_length=100):
    """Find the longest initial static segment using variance thresholding.

    Parameters
    ----------
    accel_data : ndarray, shape (N,3)
        Raw accelerometer samples.
    gyro_data : ndarray, shape (N,3)
        Raw gyroscope samples.
    window_size : int, optional
        Samples per rolling window.
    accel_var_thresh : float, optional
        Variance threshold for accelerometer (m/s^2)^2.
    gyro_var_thresh : float, optional
        Variance threshold for gyroscope (rad/s)^2.
    min_length : int, optional
        Minimum length in samples of the static segment.

    Returns
    -------
    tuple of int
        (start_idx, end_idx) indices of the detected static interval.
    """
    N = len(accel_data)
    if N < window_size:
        raise ValueError("window_size larger than data length")

    accel_var = np.array([np.var(accel_data[i:i+window_size], axis=0)
                          for i in range(N - window_size)])
    gyro_var = np.array([np.var(gyro_data[i:i+window_size], axis=0)
                         for i in range(N - window_size)])
    max_accel_var = np.max(accel_var, axis=1)
    max_gyro_var = np.max(gyro_var, axis=1)

    static_mask = (max_accel_var < accel_var_thresh) & (max_gyro_var < gyro_var_thresh)

    from itertools import groupby
    from operator import itemgetter
    groups = []
    for k, g in groupby(enumerate(static_mask), key=itemgetter(1)):
        if k:
            g = list(g)
            i0 = g[0][0]
            i1 = g[-1][0] + window_size
            groups.append((i0, i1))

    # pick the longest static group that satisfies the minimum length
    longest = (0, window_size)
    for i0, i1 in groups:
        if i1 - i0 >= min_length and i1 - i0 > longest[1] - longest[0]:
            longest = (i0, i1)

    return longest


def is_static(accel_win, gyro_win, accel_var_thresh=0.01, gyro_var_thresh=1e-6):
    """Check if the current IMU window is static."""
    return (np.max(np.var(accel_win, axis=0)) < accel_var_thresh and
            np.max(np.var(gyro_win, axis=0)) < gyro_var_thresh)

# Additional utilities for logging and thresholding
import logging


def log_static_zupt_params(static_start: int, static_end: int,
                           zupt_count: int, threshold: float,
                           static_detected: bool = True) -> None:
    """Log details about the detected static window and ZUPT events."""
    if not static_detected:
        logging.warning("No static segment detected!")
    else:
        logging.info(
            f"Static window: {static_start} to {static_end} "
            f"({static_end-static_start} samples)"
        )
    logging.info(f"ZUPT threshold used: {threshold}")
    logging.info(f"Total ZUPT events: {zupt_count}")


def save_static_zupt_params(filename: str, static_start: int, static_end: int,
                             zupt_count: int, threshold: float) -> None:
    """Save static/ZUPT parameters to a text file."""
    with open(filename, 'w') as f:
        f.write(f"Static start: {static_start}\n")
        f.write(f"Static end: {static_end}\n")
        f.write(f"Static length: {static_end-static_start}\n")
        f.write(f"ZUPT threshold: {threshold}\n")
        f.write(f"Total ZUPT events: {zupt_count}\n")


def adaptive_zupt_threshold(accel_data: np.ndarray, gyro_data: np.ndarray,
                            factor: float = 3.0) -> Tuple[float, float]:
    """Return adaptive thresholds based on early-sample statistics."""
    norm_accel = np.linalg.norm(accel_data, axis=1)
    norm_gyro = np.linalg.norm(gyro_data, axis=1)
    base = min(400, len(norm_accel))
    accel_thresh = np.mean(norm_accel[:base]) + factor * np.std(norm_accel[:base])
    gyro_thresh = np.mean(norm_gyro[:base]) + factor * np.std(norm_gyro[:base])
    return accel_thresh, gyro_thresh


def save_mat(filename: str, data: dict) -> None:
    """Save *data* dictionary to a MATLAB ``.mat`` file."""
    from scipy.io import savemat
    savemat(filename, data)


def compute_C_ECEF_to_NED(lat: float, lon: float) -> np.ndarray:
    """Return rotation matrix from ECEF to NED frame.

    Parameters
    ----------
    lat : float
        Geodetic latitude in radians.
    lon : float
        Longitude in radians.

    Returns
    -------
    ndarray of shape (3, 3)
        Rotation matrix from ECEF to NED.
    """
    s_lat = np.sin(lat)
    c_lat = np.cos(lat)
    s_lon = np.sin(lon)
    c_lon = np.cos(lon)
    return np.array([
        [-s_lat * c_lon, -s_lat * s_lon, c_lat],
        [-s_lon, c_lon, 0.0],
        [-c_lat * c_lon, -c_lat * s_lon, -s_lat],
    ])


def ecef_to_ned(pos_ecef: np.ndarray, ref_lat: float, ref_lon: float,
                ref_ecef: np.ndarray) -> np.ndarray:
    """Convert ECEF coordinates to NED frame relative to *ref_ecef*."""
    pos_ecef = np.asarray(pos_ecef)
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    if pos_ecef.ndim == 1:
        return C @ (pos_ecef - ref_ecef)
    return np.array([C @ (p - ref_ecef) for p in pos_ecef])


def ecef_to_geodetic(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert ECEF coordinates to geodetic coordinates using WGS‑84.

    This implementation follows Bowring's iterative method which offers
    improved accuracy over the previous closed form approximation.  The
    returned latitude and longitude are given in degrees while altitude is in
    metres.
    """

    # WGS‑84 parameters
    a = 6378137.0
    e_sq = 6.69437999014e-3
    b = a * np.sqrt(1.0 - e_sq)
    ep_sq = (a**2 - b**2) / b**2

    p = np.sqrt(x * x + y * y)
    if p == 0:
        lon = 0.0
    else:
        lon = np.arctan2(y, x)

    theta = np.arctan2(z * a, p * b)
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    lat = np.arctan2(z + ep_sq * b * sin_t**3, p - e_sq * a * cos_t**3)

    # Radius of curvature in the prime vertical
    N = a / np.sqrt(1.0 - e_sq * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N

    return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float = 0.0) -> Tuple[float, float, float]:
    """Return ECEF coordinates for geodetic ``lat_deg``/``lon_deg``.

    Parameters
    ----------
    lat_deg, lon_deg : float
        Latitude and longitude in degrees.
    alt : float, optional
        Altitude above the ellipsoid in metres, defaults to 0.

    Returns
    -------
    tuple of float
        ``(x, y, z)`` ECEF coordinates in metres.
    """

    a = 6378137.0
    e_sq = 6.69437999014e-3

    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    N = a / np.sqrt(1.0 - e_sq * np.sin(lat) ** 2)
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1.0 - e_sq) + alt) * np.sin(lat)
    return float(x), float(y), float(z)
