"""Utility functions for velocity estimation."""
from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter


def derive_velocity(
    time_s: np.ndarray,
    pos: np.ndarray,
    window_length: int = 11,
    polyorder: int = 2,
) -> np.ndarray:
    """Estimate velocity from position.

    Parameters
    ----------
    time_s : ndarray of shape (N,)
        Time samples in seconds.
    pos : ndarray of shape (N, 3)
        Position samples in meters (ECEF or NED).
    window_length : int, optional
        Odd window length for Savitzky--Golay smoothing.
    polyorder : int, optional
        Polynomial order for the smoothing filter.

    Returns
    -------
    ndarray of shape (N, 3)
        Estimated velocity in m/s.

    The position is smoothed with a Savitzky--Golay filter and then
    differentiated using a central difference scheme. ``window_length`` must be
    odd.

    This implementation mirrors ``MATLAB/derive_velocity.m`` for parity.
    """
    if window_length % 2 == 0:
        window_length += 1

    pos_sm = savgol_filter(pos, window_length, polyorder, axis=0)
    vel = np.zeros_like(pos_sm)
    dt = np.diff(time_s)
    vel[1:-1] = (pos_sm[2:] - pos_sm[:-2]) / (dt[1:, None] + dt[:-1, None])
    vel[0] = (pos_sm[1] - pos_sm[0]) / dt[0]
    vel[-1] = (pos_sm[-1] - pos_sm[-2]) / dt[-1]
    return vel
