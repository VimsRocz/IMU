import numpy as np
from src.gnss_imu_fusion.init_vectors import triad_basis


def test_triad_basis_orthonormal():
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    M = triad_basis(v1, v2)
    # First column must align with v1
    assert np.allclose(M[:, 0], v1)
    # Columns should be orthonormal
    I = M.T @ M
    assert np.allclose(I, np.eye(3), atol=1e-7)


def test_triad_basis_collinear():
    v1 = np.array([0.0, 0.0, 1.0])
    v2 = np.array([0.0, 0.0, 2.0])  # collinear
    M = triad_basis(v1, v2)
    # Should still be orthonormal
    I = M.T @ M
    assert np.allclose(I, np.eye(3), atol=1e-7)
    assert np.allclose(M[:, 0], np.array([0.0, 0.0, 1.0]))
