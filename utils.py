import numpy as np


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
    for i0, i1 in groups:
        if i1 - i0 >= min_length:
            return i0, i1
    return 0, window_size


def is_static(accel_win, gyro_win, accel_var_thresh=0.01, gyro_var_thresh=1e-6):
    """Check if the current IMU window is static."""
    return (np.max(np.var(accel_win, axis=0)) < accel_var_thresh and
            np.max(np.var(gyro_win, axis=0)) < gyro_var_thresh)
