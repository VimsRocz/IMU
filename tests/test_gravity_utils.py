import pytest
from src.utils import normal_gravity, validate_gravity_vector

np = pytest.importorskip("numpy")


def test_normal_gravity_specific():
    g = normal_gravity(np.deg2rad(-31.871), 159.5)
    assert np.isclose(g, 9.794245, atol=1e-6)


def test_validate_gravity_vector_specific():
    g_vec = validate_gravity_vector(-31.871, 159.5)
    expected = np.array([0.0, 0.0, 9.794245])
    assert np.allclose(g_vec, expected, atol=1e-6)
