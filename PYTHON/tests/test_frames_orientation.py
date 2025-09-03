import pytest
from src.utils import compute_C_ECEF_to_NED
from src.utils.frames import _rotation_ecef_to_ned

np = pytest.importorskip("numpy")


@pytest.mark.parametrize("lat_deg, lon_deg", [
    (0.0, 0.0),
    (45.0, 45.0),
    (-30.0, 120.0),
])
def test_compute_C_ECEF_to_NED_right_handed_down(lat_deg, lon_deg):
    lat_rad = np.radians(lat_deg)
    lon_rad = np.radians(lon_deg)
    C = compute_C_ECEF_to_NED(lat_rad, lon_rad)

    # Orthonormal and right-handed
    assert np.allclose(C @ C.T, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(C), 1.0, atol=1e-6)

    # +Z down: local up vector maps to negative NED Z
    up = np.array([
        np.cos(lat_rad) * np.cos(lon_rad),
        np.cos(lat_rad) * np.sin(lon_rad),
        np.sin(lat_rad),
    ])
    assert np.allclose(C @ up, [0.0, 0.0, -1.0], atol=1e-6)

    # Right-handed check via cross product
    assert np.allclose(np.cross(C[0], C[1]), C[2], atol=1e-6)


@pytest.mark.parametrize("lat_deg, lon_deg", [
    (0.0, 0.0),
    (45.0, 45.0),
    (-30.0, 120.0),
])
def test_rotation_ecef_to_ned_right_handed_down(lat_deg, lon_deg):
    R = _rotation_ecef_to_ned(lat_deg, lon_deg)
    lat_rad = np.radians(lat_deg)
    lon_rad = np.radians(lon_deg)

    # Orthonormal and right-handed
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)

    # +Z down: third column points toward Earth
    up = np.array([
        np.cos(lat_rad) * np.cos(lon_rad),
        np.cos(lat_rad) * np.sin(lon_rad),
        np.sin(lat_rad),
    ])
    assert np.allclose(R[:, 2], -up, atol=1e-6)

    # Right-handed check via cross product
    assert np.allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-6)

    # Relationship with compute_C_ECEF_to_NED
    C = compute_C_ECEF_to_NED(lat_rad, lon_rad)
    assert np.allclose(C, R.T, atol=1e-6)
