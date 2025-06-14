import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import numpy as np
from imu_fusion.attitude import (
    compute_C_ECEF_to_NED,
    rot_to_quaternion,
    quat_multiply,
    quat_normalize,
    triad,
    davenport_q_method,
    svd_method,
    estimate_initial_orientation,
)
import pandas as pd


def test_compute_C_ECEF_to_NED_identity_lat_lon_zero():
    C = compute_C_ECEF_to_NED(0.0, 0.0)
    expected = np.array([
        [0, 0, 1],
        [0, 1, 0],
        [-1, 0, 0],
    ])
    assert np.allclose(C, expected)


def test_quaternion_helpers_roundtrip():
    R = np.eye(3)
    q = rot_to_quaternion(R)
    # quaternion should represent identity rotation
    expected = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(q, expected)
    # multiplication with identity should return same quaternion
    q2 = quat_multiply(q, q)
    q2n = quat_normalize(q2)
    assert np.allclose(q2n, expected)


def test_triad_davenport_svd_identity():
    g_b = np.array([0.0, 0.0, 1.0])
    w_b = np.array([1.0, 0.0, 0.0])
    g_n = g_b.copy()
    w_n = w_b.copy()

    R_tri = triad(g_b, w_b, g_n, w_n)
    R_dav = davenport_q_method(g_b, w_b, g_n, w_n)
    R_svd = svd_method(g_b, w_b, g_n, w_n)

    I = np.eye(3)
    assert np.allclose(R_tri, I)
    assert np.allclose(R_dav, I)
    assert np.allclose(R_svd, I)


def test_estimate_initial_orientation_identity_yaw():
    imu = np.zeros((1, 6))
    gnss = pd.DataFrame({
        "Latitude_deg": [0.0],
        "Longitude_deg": [0.0],
        "VX_ECEF_mps": [1.0],
        "VY_ECEF_mps": [0.0],
        "VZ_ECEF_mps": [0.0],
    })
    q = estimate_initial_orientation(imu, gnss)
    expected = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(q, expected)

