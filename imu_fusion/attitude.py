import numpy as np


def compute_C_ECEF_to_NED(lat: float, lon: float) -> np.ndarray:
    """Compute rotation matrix from ECEF to NED frame."""
    sin_phi = np.sin(lat)
    cos_phi = np.cos(lat)
    sin_lambda = np.sin(lon)
    cos_lambda = np.cos(lon)
    return np.array([
        [-sin_phi * cos_lambda, -sin_phi * sin_lambda, cos_phi],
        [-sin_lambda, cos_lambda, 0.0],
        [-cos_phi * cos_lambda, -cos_phi * sin_lambda, -sin_phi],
    ])


def rot_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a quaternion."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)


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


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)
