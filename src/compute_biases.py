"""IMU bias estimation utilities."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def compute_biases(
    acc_body: np.ndarray,
    gyro_body: np.ndarray,
    start: int,
    end: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return mean accelerometer and gyroscope measurements over a static interval.

    Parameters
    ----------
    acc_body : ndarray, shape (N, 3)
        Accelerometer samples in the body frame (m/s^2).
    gyro_body : ndarray, shape (N, 3)
        Gyroscope samples in the body frame (rad/s).
    start : int
        Inclusive index of the static interval start.
    end : int
        Exclusive index of the static interval end.

    Returns
    -------
    tuple of ndarray
        ``(acc_bias, gyro_bias)`` arrays of shape ``(3,)``.
    """
    static_acc = acc_body[start:end]
    static_gyro = gyro_body[start:end]
    acc_bias = np.mean(static_acc, axis=0)
    gyro_bias = np.mean(static_gyro, axis=0)
    return acc_bias, gyro_bias
