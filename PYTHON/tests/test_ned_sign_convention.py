import numpy as np
from src.utils.frames import ecef_to_ned
from src.utils.ecef_llh import lla_to_ecef


def test_ned_sign_convention():
    """Verify NED conversion maintains expected sign conventions."""
    lat0_deg = 10.0
    lon0_deg = 20.0
    h0 = 0.0

    # Reference ECEF position
    lat_rad = np.radians(lat0_deg)

    # Radii of curvature for 1 m displacements
    a = 6378137.0
    e2 = 6.69437999014e-3
    R_N = a / np.sqrt(1 - e2 * np.sin(lat_rad) ** 2)
    R_E = R_N * np.cos(lat_rad)
    dlat_deg = np.degrees(1 / (R_N + h0))
    dlon_deg = np.degrees(1 / (R_E + h0))

    # Points 1 m north, east, down, and up from reference
    x_n, y_n, z_n = lla_to_ecef(lat0_deg + dlat_deg, lon0_deg, h0)
    x_e, y_e, z_e = lla_to_ecef(lat0_deg, lon0_deg + dlon_deg, h0)
    x_d, y_d, z_d = lla_to_ecef(lat0_deg, lon0_deg, h0 - 1.0)
    x_u, y_u, z_u = lla_to_ecef(lat0_deg, lon0_deg, h0 + 1.0)

    n, e, d = ecef_to_ned(
        [x_n, x_e, x_d, x_u],
        [y_n, y_e, y_d, y_u],
        [z_n, z_e, z_d, z_u],
        lat0_deg,
        lon0_deg,
        h0,
    )

    assert np.allclose(n[0], 1.0, atol=1e-2)
    assert np.allclose(e[1], 1.0, atol=1e-2)
    assert np.allclose(d[2], 1.0, atol=1e-2)
    assert np.allclose(d[3], -1.0, atol=1e-2)
