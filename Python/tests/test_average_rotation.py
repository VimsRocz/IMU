import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import pytest
np = pytest.importorskip("numpy")
from gnss_imu_fusion.init_vectors import average_rotation_matrices

def test_average_rotation_identity():
    R1 = np.eye(3)
    R2 = np.eye(3)
    R3 = np.eye(3)
    R = average_rotation_matrices([R1, R2, R3])
    assert np.allclose(R, np.eye(3), atol=1e-6)
    I = R @ R.T
    assert np.allclose(I, np.eye(3), atol=1e-6)
