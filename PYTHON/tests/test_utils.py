import pytest
from src.utils import compute_C_ECEF_to_NED, zero_base_time

np = pytest.importorskip("numpy")


def test_rotation_matrix_orthonormal():
    lat = np.deg2rad(-32.0)
    lon = np.deg2rad(133.0)
    C = compute_C_ECEF_to_NED(lat, lon)
    # Orthonormal check
    identity = C @ C.T
    assert np.allclose(identity, np.eye(3), atol=1e-6)


def test_zero_base_time():
    assert zero_base_time([]).size == 0
    t = np.array([5.0, 6.5, 7.0])
    assert np.allclose(zero_base_time(t), t - 5.0)
