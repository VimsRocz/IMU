import numpy as np

from ..earth_model import gravity_ned


def mechanise_static(acc_body: np.ndarray, gyro_body: np.ndarray, dt: float,
                      lat: float = 0.0, h: float = 0.0) -> np.ndarray:
    """Simple strapdown integration for a stationary segment.

    Parameters
    ----------
    acc_body : ndarray of shape (N, 3)
        Raw accelerometer measurements in the body frame [m/s²].
    gyro_body : ndarray of shape (N, 3)
        Gyroscope measurements (unused here).
    dt : float
        Sampling interval in seconds.
    lat : float, optional
        Latitude in radians for gravity model.
    h : float, optional
        Height above ellipsoid in metres.

    Returns
    -------
    ndarray of shape (N, 3)
        Integrated velocity in the NED frame.
    """
    g_ned = gravity_ned(lat, h)
    N = acc_body.shape[0]
    v_n = np.zeros((N, 3))
    C_bn = np.eye(3)
    accel_bias = np.zeros(3)
    coriolis_n = np.zeros(3)

    for i in range(1, N):
        # >>> GRAVITY_BEGIN
        # Rotate delta-v to navigation frame
        f_ib_n = C_bn @ (acc_body[i] - accel_bias)

        # Apply gravity **once**
        # *Subtract* because accelerometer measures specific force (i.e. +g when at rest)
        f_ib_n -= g_ned               # <-- CORRECT SIGN

        # Add Coriolis + centrifugal if present (unchanged)
        f_ib_n += coriolis_n
        # <<< GRAVITY_END

        v_n[i] = v_n[i - 1] + f_ib_n * dt

    return v_n
