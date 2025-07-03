from __future__ import annotations

import logging
from typing import Optional, Tuple

import numpy as np
import pandas as pd

from ..constants import GRAVITY, EARTH_RATE
from ..utils import detect_static_interval, ecef_to_geodetic, compute_C_ECEF_to_NED
from .init_vectors import butter_lowpass_filter


def compute_reference_vectors(
    gnss_file: str, mag_file: Optional[str] = None
) -> Tuple[
    float,
    float,
    float,
    np.ndarray,
    np.ndarray,
    Optional[np.ndarray],
    np.ndarray,
    np.ndarray,
]:
    """Return latitude/longitude and reference vectors in the NED frame."""
    gnss_data = pd.read_csv(gnss_file)
    valid_rows = gnss_data[(gnss_data["X_ECEF_m"] != 0) | (gnss_data["Y_ECEF_m"] != 0) | (gnss_data["Z_ECEF_m"] != 0)]
    if valid_rows.empty:
        raise ValueError("No valid ECEF coordinates found in GNSS data")
    row = valid_rows.iloc[0]
    x_ecef = float(row["X_ECEF_m"])
    y_ecef = float(row["Y_ECEF_m"])
    z_ecef = float(row["Z_ECEF_m"])
    lat_deg, lon_deg, alt = ecef_to_geodetic(x_ecef, y_ecef, z_ecef)
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    g_ned = np.array([0.0, 0.0, GRAVITY])
    omega_ie_ned = EARTH_RATE * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    initial_vel = row[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].values
    C_e2n = compute_C_ECEF_to_NED(lat, lon)
    initial_vel_ned = C_e2n @ initial_vel

    mag_ned = None
    if mag_file:
        try:
            mag_data = np.loadtxt(mag_file, delimiter=",")
            if mag_data.ndim == 2 and mag_data.shape[1] >= 3:
                mag_ned = np.mean(mag_data[:, :3], axis=0)
        except Exception as exc:  # pragma: no cover - optional feature
            logging.debug("Failed to load magnetometer file: %s", exc)
    return (
        lat_deg,
        lon_deg,
        alt,
        g_ned,
        omega_ie_ned,
        mag_ned,
        initial_vel_ned,
        np.array([x_ecef, y_ecef, z_ecef]),
    )


def measure_body_vectors(
    imu_file: str,
    static_start: Optional[int] = None,
    static_end: Optional[int] = None,
    mag_file: Optional[str] = None,
) -> Tuple[float, np.ndarray, np.ndarray, Optional[np.ndarray]]:
    """Estimate gravity and Earth rotation in the body frame."""
    data = np.loadtxt(imu_file)
    if data.shape[1] < 10:
        raise ValueError("Unexpected IMU data format")
    time = data[:, 1]
    gyro = data[:, 2:5]
    acc = data[:, 5:8]

    if len(time) > 1:
        dt = float(np.mean(np.diff(time[:100])))
    else:
        dt = 1.0 / 400.0
    gyro /= dt
    acc /= dt
    gyro = butter_lowpass_filter(gyro)
    acc = butter_lowpass_filter(acc)

    if static_start is None:
        static_start, static_end = detect_static_interval(
            acc,
            gyro,
            window_size=80,
            accel_var_thresh=0.01,
            gyro_var_thresh=1e-6,
            min_length=80,
        )
    else:
        static_start = max(0, static_start)
        static_end = min(static_end or len(acc), len(acc))
    static_acc = np.mean(acc[static_start:static_end], axis=0)
    static_gyro = np.mean(gyro[static_start:static_end], axis=0)
    scale = GRAVITY / np.linalg.norm(static_acc)
    acc *= scale
    static_acc *= scale
    g_body = -static_acc
    omega_ie_body = static_gyro

    mag_body = None
    if mag_file:
        try:
            mag_data = np.loadtxt(mag_file, delimiter=",")
            if mag_data.ndim == 2 and mag_data.shape[1] >= 3:
                mag_body = np.mean(mag_data[:, :3], axis=0)
        except Exception as exc:  # pragma: no cover - optional feature
            logging.debug("Failed to load magnetometer file: %s", exc)
    return dt, g_body, omega_ie_body, mag_body
