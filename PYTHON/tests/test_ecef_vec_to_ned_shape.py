import pytest

np = pytest.importorskip("numpy")

from src.utils.frames import ecef_vec_to_ned


def test_ecef_vec_to_ned_handles_multiple_samples():
    lat = np.deg2rad(10.0)
    lon = np.deg2rad(20.0)
    vec = np.arange(15.0).reshape(5, 3)
    ned = ecef_vec_to_ned(vec, lat, lon)
    assert ned.shape == (5, 3)
    t = np.linspace(0.0, 1.0, 5)
    assert np.allclose(np.interp(t, t, ned[:, 0]), ned[:, 0])
