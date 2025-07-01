import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import pytest
np = pytest.importorskip("numpy")
from utils import compute_C_ECEF_to_NED


def test_rotation_matrix_orthonormal():
    lat = np.deg2rad(-32.0)
    lon = np.deg2rad(133.0)
    C = compute_C_ECEF_to_NED(lat, lon)
    # Orthonormal check
    I = C @ C.T
    assert np.allclose(I, np.eye(3), atol=1e-6)
