import pytest
from src.utils import (
    adaptive_zupt_threshold,
    save_static_zupt_params,
    ecef_to_ned,
    compute_C_ECEF_to_NED,
)

np = pytest.importorskip("numpy")


def test_adaptive_zupt_threshold_constant():
    accel = np.ones((500, 3))
    gyro = np.ones((500, 3)) * 0.5
    a_t, g_t = adaptive_zupt_threshold(accel, gyro, factor=2.0)
    assert np.isclose(a_t, np.sqrt(3))
    assert np.isclose(g_t, np.sqrt(3) * 0.5)


def test_save_static_zupt_params(tmp_path):
    f = tmp_path / "params.txt"
    save_static_zupt_params(f, 10, 20, 3, 0.4)
    content = f.read_text()
    assert "Static start: 10" in content
    assert "Static end: 20" in content
    assert "Static length: 10" in content
    assert "ZUPT threshold: 0.4" in content
    assert "Total ZUPT events: 3" in content


def test_ecef_to_ned_single_vector():
    ref_lat = np.deg2rad(-32.0)
    ref_lon = np.deg2rad(133.0)
    pos_ecef = np.array([1234.0, 5678.0, 91011.0])
    ref_ecef = np.array([1000.0, 2000.0, 3000.0])
    ned = ecef_to_ned(pos_ecef, ref_lat, ref_lon, ref_ecef)
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    expected = C @ (pos_ecef - ref_ecef)
    assert np.allclose(ned, expected)


def test_ecef_to_ned_multi_vector():
    ref_lat = np.deg2rad(12.5)
    ref_lon = np.deg2rad(-45.0)
    pos_ecef = np.array([
        [1000.0, 2000.0, 3000.0],
        [1100.0, 2100.0, 3200.0],
    ])
    ref_ecef = np.array([900.0, 1800.0, 2900.0])
    ned = ecef_to_ned(pos_ecef, ref_lat, ref_lon, ref_ecef)
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    expected = np.array([C @ (p - ref_ecef) for p in pos_ecef])
    assert np.allclose(ned, expected)
