import importlib
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "src"))

np = pytest.importorskip("numpy")
compute_C_ECEF_to_NED = importlib.import_module("utils").compute_C_ECEF_to_NED


def test_rotation_matrix_orthonormal():
    lat = np.deg2rad(-32.0)
    lon = np.deg2rad(133.0)
    C = compute_C_ECEF_to_NED(lat, lon)
    # Orthonormal check
    identity = C @ C.T
    assert np.allclose(identity, np.eye(3), atol=1e-6)
