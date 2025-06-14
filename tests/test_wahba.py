import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import numpy as np
from imu_fusion.wahba import triad_method, davenport_q_method, svd_method
from imu_fusion.attitude import rot_to_quaternion


# Known vector pairs corresponding to a 90 degree yaw rotation
v1_b = np.array([1.0, 0.0, 0.0])
v2_b = np.array([0.0, 1.0, 0.0])
v1_n = np.array([0.0, 1.0, 0.0])
v2_n = np.array([-1.0, 0.0, 0.0])
expected_R = np.array([
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])


def _check_rotation_matrix(R: np.ndarray) -> None:
    # Orthogonality and determinant close to 1
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-7)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-7)


def test_triad_method_returns_valid_rotation():
    R = triad_method(v1_b, v2_b, v1_n, v2_n)
    _check_rotation_matrix(R)
    assert np.allclose(R, expected_R)


def test_davenport_q_method_returns_valid_rotation():
    R = davenport_q_method(v1_b, v2_b, v1_n, v2_n)
    _check_rotation_matrix(R)
    assert np.allclose(R, expected_R)


def test_svd_method_returns_valid_rotation():
    R = svd_method(v1_b, v2_b, v1_n, v2_n)
    _check_rotation_matrix(R)
    assert np.allclose(R, expected_R)


# Quaternion tests for simple rotation matrices

def test_rot_to_quaternion_yaw_90():
    q = rot_to_quaternion(expected_R)
    expected_q = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
    assert np.allclose(q, expected_q)


def test_rot_to_quaternion_roll_90():
    R = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ]
    )
    q = rot_to_quaternion(R)
    expected_q = np.array([np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0])
    assert np.allclose(q, expected_q)
