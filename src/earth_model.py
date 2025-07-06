import numpy as np

from .utils import normal_gravity


def gravity_ned(lat_rad: float, h_m: float = 0.0) -> np.ndarray:
    """Return gravity vector in **NED** (+Down) [m/s²]."""
    g = normal_gravity(lat_rad, h_m)
    return np.array([0.0, 0.0, +g])   # +g is down in NED
