import pytest

np = pytest.importorskip("numpy")
from src.gnss_imu_fusion.init_vectors import basic_butterworth_filter


def test_basic_butterworth_filter_effect():
    fs = 100.0
    t = np.linspace(0, 1, int(fs) + 1)
    sig = np.sin(2 * np.pi * 1 * t) + 0.5 * np.sin(2 * np.pi * 30 * t)
    pure = np.sin(2 * np.pi * 1 * t)
    filt = basic_butterworth_filter(sig, cutoff=5.0, fs=fs, order=4)
    err_before = np.sqrt(np.mean((sig - pure) ** 2))
    err_after = np.sqrt(np.mean((filt - pure) ** 2))
    assert err_after < err_before
    assert filt.shape == sig.shape
