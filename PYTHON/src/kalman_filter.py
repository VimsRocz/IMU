from __future__ import annotations

import numpy as np
from filterpy.kalman import KalmanFilter


def init_bias_kalman(
    dt: float,
    meas_R_pos: np.ndarray,
    meas_R_vel: np.ndarray,
    vel_q_scale: float = 1.0,
) -> KalmanFilter:
    """Return Kalman filter with gyro bias states.

    State vector: ``[pos(3), vel(3), quat(4), gyro_bias(3)]``.
    ``dt``
        Sample period in seconds.
    ``meas_R_pos``, ``meas_R_vel``
        3x3 measurement covariance blocks for position and velocity.
    ``vel_q_scale``
        Multiplier applied to the velocity process noise block.
    """
    kf = KalmanFilter(dim_x=13, dim_z=6, dim_u=3)
    kf.F = np.eye(13)
    kf.F[0:3, 3:6] = np.eye(3) * dt

    kf.B = np.zeros((13, 3))
    kf.B[0:3] = 0.5 * dt * dt * np.eye(3)
    kf.B[3:6] = dt * np.eye(3)

    kf.H = np.zeros((6, 13))
    kf.H[0:3, 0:3] = np.eye(3)
    kf.H[3:6, 3:6] = np.eye(3)

    kf.P *= 1.0
    kf.Q = np.eye(13) * 0.01
    kf.Q[3:6, 3:6] *= vel_q_scale
    kf.Q[10:13, 10:13] = np.eye(3) * 1e-8
    kf.R = np.block(
        [[meas_R_pos, np.zeros((3, 3))], [np.zeros((3, 3)), meas_R_vel]]
    )
    return kf


def inject_zupt(kf: KalmanFilter) -> None:
    """Inject Zero-Velocity pseudo-measurement."""
    # Measurement model: z = H x + v, here z = [0,0,0] for velocity components
    H = np.hstack([np.zeros((3, 3)), np.eye(3), np.zeros((3, 7))])  # (3,13)
    R = np.eye(3) * 1e-4
    z = np.zeros(3)  # use 1D vector for consistent shapes
    # Innovation and Kalman gain
    y = z - (H @ kf.x).reshape(3,)
    S = H @ kf.P @ H.T + R
    K = kf.P @ H.T @ np.linalg.inv(S)
    # State/covariance update (keep x as 1D vector)
    kf.x = (kf.x + K @ y).reshape(-1)
    kf.P = (np.eye(kf.dim_x) - K @ H) @ kf.P
