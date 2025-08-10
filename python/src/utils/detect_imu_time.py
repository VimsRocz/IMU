"""Placeholder for detect_imu_time helper (MATLAB parity).

This stub mirrors the MATLAB ``detect_imu_time`` utility, which constructs a
monotonic IMU time vector handling sub-second rollovers. The Python
implementation is pending; for now it returns a simple constant-rate time
vector using ``dt_fallback``.
"""
from __future__ import annotations

import numpy as np


def detect_imu_time(imu_mat: np.ndarray, dt_fallback: float = 0.0025) -> np.ndarray:
    """Generate a basic time vector for IMU samples.

    Parameters
    ----------
    imu_mat : np.ndarray
        Raw IMU matrix as loaded from a file.
    dt_fallback : float, optional
        Fallback sample interval in seconds, default 0.0025 (~400 Hz).

    Returns
    -------
    np.ndarray
        Monotonic time vector assuming a constant sample rate.
    """
    n = imu_mat.shape[0]
    return np.arange(n) * dt_fallback
