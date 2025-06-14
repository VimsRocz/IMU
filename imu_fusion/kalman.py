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


def kalman_with_bias(
    zs: np.ndarray,
    F: np.ndarray,
    H: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    x0: np.ndarray,
    *,
    initial_bias: np.ndarray | None = None,
    bias_process_var: float = 1e-5,
) -> tuple[np.ndarray, np.ndarray]:
    """Kalman filter with three additional accelerometer-bias states."""
    base_dim = len(x0)
    dim_x = base_dim + 3
    kf = KalmanFilter(dim_x=dim_x, dim_z=zs.shape[1])
    if initial_bias is None:
        initial_bias = np.zeros(3)
    if initial_bias.shape != (3,):
        raise ValueError("initial_bias must have shape (3,)")
    kf.x = np.hstack([x0, initial_bias])

    F_ext = np.eye(dim_x)
    F_ext[:base_dim, :base_dim] = F
    kf.F = F_ext

    H_ext = np.zeros((H.shape[0], dim_x))
    H_ext[:, :H.shape[1]] = H
    kf.H = H_ext

    Q_ext = np.eye(dim_x) * bias_process_var
    Q_ext[:base_dim, :base_dim] = Q
    kf.Q = Q_ext
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


def residual_diagnostics(residuals: np.ndarray, threshold: float = 100.0) -> bool:
    """Print a warning if position residual RMS exceeds ``threshold``."""
    if residuals.size == 0:
        return False
    rms = np.sqrt(np.mean(residuals[:, :3] ** 2, axis=0))
    if np.any(rms > threshold):
        print(f"WARNING: residual RMS exceeds {threshold} m: {rms}")
        return True
    return False
