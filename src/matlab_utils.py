"""Python stubs for the MATLAB TRIAD pipeline."""

from __future__ import annotations

import numpy as np


def read_imu(path: str):
    """Stub for MATLAB :func:`read_imu`."""
    raise NotImplementedError("read_imu is implemented in MATLAB")


def read_gnss(path: str):
    """Stub for MATLAB :func:`read_gnss`."""
    raise NotImplementedError("read_gnss is implemented in MATLAB")


def ecef2ned_matrix(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Stub for MATLAB :func:`ecef2ned_matrix`."""
    raise NotImplementedError("ecef2ned_matrix is implemented in MATLAB")


def triad_algorithm(acc_body: np.ndarray, mag_body: np.ndarray, vel_ned: np.ndarray):
    """Stub for MATLAB :func:`triad_algorithm`."""
    raise NotImplementedError("triad_algorithm is implemented in MATLAB")


def save_attitude_output(out_dir: str, R_bn: np.ndarray, q_bn: np.ndarray):
    """Stub for MATLAB :func:`save_attitude_output`."""
    raise NotImplementedError("save_attitude_output is implemented in MATLAB")


def plot_initial_orientation(out_dir: str, R_bn: np.ndarray):
    """Stub for MATLAB :func:`plot_initial_orientation`."""
    raise NotImplementedError("plot_initial_orientation is implemented in MATLAB")

