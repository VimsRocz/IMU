import numpy as np
import pandas as pd


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


def estimate_initial_orientation(imu: np.ndarray, gnss: pd.DataFrame) -> np.ndarray:
    """Estimate initial body-to-NED quaternion from GNSS velocity."""
    lat = np.deg2rad(float(gnss.iloc[0].get("Latitude_deg", 0.0)))
    lon = np.deg2rad(float(gnss.iloc[0].get("Longitude_deg", 0.0)))
    vel_cols = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
    if set(vel_cols) <= set(gnss.columns):
        v_ecef = gnss.iloc[0][vel_cols].to_numpy(float)
    else:
        v_ecef = np.zeros(3)
    C = compute_C_ECEF_to_NED(lat, lon)
    v_ned = C @ v_ecef
    yaw = float(np.arctan2(v_ned[1], v_ned[0]))
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    R = np.array(
        [
            [cy, sy, 0.0],
            [-sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rot_to_quaternion(R)


def triad(v1_b: np.ndarray, v2_b: np.ndarray, v1_n: np.ndarray, v2_n: np.ndarray) -> np.ndarray:
    """Compute body-to-navigation rotation matrix using the TRIAD algorithm."""
    t1_b = v1_b / np.linalg.norm(v1_b)
    t2_b_temp = np.cross(t1_b, v2_b)
    if np.linalg.norm(t2_b_temp) < 1e-10:
        t2_b = np.array([1.0, 0.0, 0.0])
    else:
        t2_b = t2_b_temp / np.linalg.norm(t2_b_temp)
    t3_b = np.cross(t1_b, t2_b)

    t1_n = v1_n / np.linalg.norm(v1_n)
    t2_n_temp = np.cross(t1_n, v2_n)
    if np.linalg.norm(t2_n_temp) < 1e-10:
        t2_n = np.array([1.0, 0.0, 0.0])
    else:
        t2_n = t2_n_temp / np.linalg.norm(t2_n_temp)
    t3_n = np.cross(t1_n, t2_n)

    R = np.column_stack((t1_n, t2_n, t3_n)) @ np.column_stack((t1_b, t2_b, t3_b)).T
    return R


def davenport_q_method(
    v1_b: np.ndarray,
    v2_b: np.ndarray,
    v1_n: np.ndarray,
    v2_n: np.ndarray,
    w1: float = 0.9999,
    w2: float = 0.0001,
) -> np.ndarray:
    """Compute body-to-navigation rotation matrix using Davenport's Q-method."""
    B = w1 * np.outer(v1_n, v1_b) + w2 * np.outer(v2_n, v2_b)
    sigma = np.trace(B)
    S = B + B.T
    Z = np.array([B[1, 2] - B[2, 1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]])
    K = np.zeros((4, 4))
    K[0, 0] = sigma
    K[0, 1:] = Z
    K[1:, 0] = Z
    K[1:, 1:] = S - sigma * np.eye(3)
    eigvals, eigvecs = np.linalg.eigh(K)
    q = eigvecs[:, np.argmax(eigvals)]
    if q[0] < 0:
        q = -q
    q = np.array([q[0], -q[1], -q[2], -q[3]])
    return quaternion_to_rot(q)


def svd_method(
    v1_b: np.ndarray,
    v2_b: np.ndarray,
    v1_n: np.ndarray,
    v2_n: np.ndarray,
    w1: float = 0.9999,
    w2: float = 0.0001,
) -> np.ndarray:
    """Compute body-to-navigation rotation matrix using the SVD method."""
    M = w1 * np.outer(v1_n, v1_b) + w2 * np.outer(v2_n, v2_b)
    U, _, Vt = np.linalg.svd(M)
    R = U @ np.diag([1, 1, np.linalg.det(U) * np.linalg.det(Vt)]) @ Vt
    return R


def quaternion_to_rot(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to rotation matrix."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)],
    ])
