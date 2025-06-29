"""Kalman filter utilities extracted from the original script."""

import numpy as np


def quat_multiply(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Quaternion multiplication."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


def quat_from_rate(omega: np.ndarray, dt: float) -> np.ndarray:
    """Quaternion from angular rate."""
    theta = np.linalg.norm(omega) * dt
    if theta == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    half = theta / 2.0
    return np.array([np.cos(half), *(np.sin(half) * axis)])


def quat2euler(q: np.ndarray) -> tuple[float, float, float]:
    """Return roll, pitch, yaw (rad) from w-x-y-z quaternion."""
    w, x, y, z = q
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return roll, pitch, yaw

