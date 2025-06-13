import numpy as np
from filterpy.kalman import KalmanFilter


def simple_kalman(
    zs: np.ndarray, F: np.ndarray, H: np.ndarray, Q: np.ndarray, R: np.ndarray, x0: np.ndarray
) -> np.ndarray:
    """Run a simple Kalman Filter."""
    kf = KalmanFilter(dim_x=len(x0), dim_z=zs.shape[1])
    kf.x = x0
    kf.F = F
    kf.H = H
    kf.Q = Q
    kf.R = R
    xs = []
    for z in zs:
        kf.predict()
        kf.update(z)
        xs.append(kf.x.copy())
    return np.array(xs)


def kalman_with_residuals(
    zs: np.ndarray, F: np.ndarray, H: np.ndarray, Q: np.ndarray, R: np.ndarray, x0: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Run Kalman Filter and return states and residuals."""
    kf = KalmanFilter(dim_x=len(x0), dim_z=zs.shape[1])
    kf.x = x0
    kf.F = F
    kf.H = H
    kf.Q = Q
    kf.R = R
    xs = []
    residuals = []
    for z in zs:
        kf.predict()
        residual = z - (kf.H @ kf.x)
        kf.update(z)
        xs.append(kf.x.copy())
        residuals.append(residual)
    return np.array(xs), np.array(residuals)
