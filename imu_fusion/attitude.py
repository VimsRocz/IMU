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

from .wahba import triad_method


def estimate_initial_orientation_triad(
    accel: np.ndarray,
    gyro: np.ndarray,
    gnss: pd.DataFrame,
    samples: int = 4000,
) -> np.ndarray:
    """Estimate initial quaternion using the TRIAD method with gravity normalisation."""
    lat = np.deg2rad(float(gnss.iloc[0].get("Latitude_deg", 0.0)))
    lon = np.deg2rad(float(gnss.iloc[0].get("Longitude_deg", 0.0)))
    g_ned = np.array([0.0, 0.0, 9.81])
    omega_ie = 7.2921159e-5 * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    n = min(samples, len(accel))
    g_body = -np.mean(accel[:n], axis=0)
    if np.linalg.norm(g_body) > 1e-3:
        g_body = 9.81 * g_body / np.linalg.norm(g_body)
    omega_body = np.mean(gyro[:n], axis=0)
    R = triad_method(g_body, omega_body, g_ned, omega_ie)
    return rot_to_quaternion(R)
