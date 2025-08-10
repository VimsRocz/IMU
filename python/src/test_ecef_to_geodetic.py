import shutil
import subprocess
import pytest
from src.utils import ecef_to_geodetic

np = pytest.importorskip("numpy")

# Helper to compute ECEF from geodetic coordinates for testing
def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    a = 6378137.0
    e_sq = 6.69437999014e-3
    N = a / np.sqrt(1 - e_sq * np.sin(lat)**2)
    x = (N + alt_m) * np.cos(lat) * np.cos(lon)
    y = (N + alt_m) * np.cos(lat) * np.sin(lon)
    z = ((1 - e_sq) * N + alt_m) * np.sin(lat)
    return x, y, z

@pytest.mark.parametrize(
    "lat_deg,lon_deg,alt", [
        (0.0, 0.0, 0.0),
        (45.0, 45.0, 1000.0),
        (-32.026554, 133.455801, 120.0),
    ],
)
def test_roundtrip(lat_deg, lon_deg, alt):
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
    lat, lon, alt_out = ecef_to_geodetic(x, y, z)
    assert np.allclose([lat, lon, alt_out], [lat_deg, lon_deg, alt], atol=1e-6)

def test_matches_matlab():
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    lat_deg, lon_deg, alt = 20.3, -45.1, 250.0
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
    cmd = (
        "addpath('MATLAB');"
        f"[la,lo,al]=ecef2geodetic({x},{y},{z});"
        "fprintf('%.10f %.10f %.4f\\n',la,lo,al);"
    )
    out = subprocess.check_output([matlab, "-batch", cmd], text=True)
    m_lat, m_lon, m_alt = [float(v) for v in out.strip().split()]
    p_lat, p_lon, p_alt = ecef_to_geodetic(x, y, z)
    assert np.allclose([p_lat, p_lon, p_alt], [m_lat, m_lon, m_alt], atol=1e-6)
