"""TRIAD basis construction utilities.

This module mirrors :func:`triad_matrix` in the MATLAB utilities.
"""
from __future__ import annotations

import numpy as np


def triad_matrix(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """Return a 3x3 orthonormal basis from two 3-vectors.

    Parameters
    ----------
    v1, v2 : ndarray, shape (3,)
        Primary and secondary direction vectors.

    Returns
    -------
    ndarray, shape (3, 3)
        Columns ``[t1, t2, t3]`` forming a right-handed basis.

    Raises
    ------
    ValueError
        If vectors are near zero or collinear.
    """

    v1 = np.asarray(v1).reshape(3)
    v2 = np.asarray(v2).reshape(3)
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-12:
        raise ValueError("triad_matrix: primary vector near zero")
    if n2 < 1e-12:
        raise ValueError("triad_matrix: secondary vector near zero")

    t1 = v1 / n1
    v2p = v2 - t1 * np.dot(t1, v2)
    n2p = np.linalg.norm(v2p)
    if n2p < 1e-12:
        raise ValueError("triad_matrix: vectors are collinear; cannot build basis")
    t2 = v2p / n2p
    t3 = np.cross(t1, t2)
    t3 /= np.linalg.norm(t3)
    T = np.column_stack((t1, t2, t3))
    if abs(np.linalg.det(T)) < 1e-6:
        raise ValueError("triad_matrix: degenerate basis (det ~ 0)")
    return T

