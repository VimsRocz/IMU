from utils import frames as _frames
import numpy as np

R_ecef_to_ned = _frames.R_ecef_to_ned


def test_ecef_to_ned_equator_prime_meridian():
    lat = 0.0
    lon = 0.0
    R = R_ecef_to_ned(lat, lon)
    ex = np.array([1.0, 0.0, 0.0])
    ey = np.array([0.0, 1.0, 0.0])
    ez = np.array([0.0, 0.0, 1.0])
    assert np.allclose(R @ ex, [0.0, 0.0, -1.0], atol=1e-12)
    assert np.allclose(R @ ey, [0.0, 1.0, 0.0], atol=1e-12)
    assert np.allclose(R @ ez, [1.0, 0.0, 0.0], atol=1e-12)
