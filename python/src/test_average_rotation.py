import pytest
from src.gnss_imu_fusion.init_vectors import average_rotation_matrices

np = pytest.importorskip("numpy")

def test_average_rotation_identity():
    R1 = np.eye(3)
    R2 = np.eye(3)
    R3 = np.eye(3)
    R = average_rotation_matrices([R1, R2, R3])
    assert np.allclose(R, np.eye(3), atol=1e-6)
    identity = R @ R.T
    assert np.allclose(identity, np.eye(3), atol=1e-6)
