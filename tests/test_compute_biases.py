import pytest
from src.compute_biases import compute_biases

np = pytest.importorskip("numpy")


def test_compute_biases_basic():
    rng = np.random.default_rng(0)
    acc = rng.normal(0, 0.1, size=(100, 3))
    gyro = rng.normal(0, 0.01, size=(100, 3))
    start, end = 10, 60
    a_b, g_b = compute_biases(acc, gyro, start, end)
    assert np.allclose(a_b, acc[start:end].mean(axis=0))
    assert np.allclose(g_b, gyro[start:end].mean(axis=0))
