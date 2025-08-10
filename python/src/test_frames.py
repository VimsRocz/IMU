import importlib.util
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location(
    "_frames", Path(__file__).resolve().parents[1] / "src" / "utils" / "frames.py"
)
frames = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frames)
R_ecef_to_ned = frames.R_ecef_to_ned


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
