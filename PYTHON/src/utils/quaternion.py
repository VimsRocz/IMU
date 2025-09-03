"""Quaternion utilities for repository-wide convention.

All quaternions are expected to be in scalar-last ``[x, y, z, w]`` format and
represent the rotation from the body frame to the world frame.
"""
from __future__ import annotations

import numpy as np


def assert_quaternion_convention(q: np.ndarray) -> None:
    """Validate quaternion shape, normalization and ordering.

    Parameters
    ----------
    q : np.ndarray
        Quaternion or array of quaternions in ``[x, y, z, w]`` order.

    Raises
    ------
    AssertionError
        If the quaternion is not ``(..., 4)`` shaped, not normalised, or clearly
        not using ``[x, y, z, w]`` order (e.g. identity with scalar first).
    """
    q = np.asarray(q, dtype=float)
    if q.shape[-1] != 4:
        raise AssertionError("Quaternion must have shape (..., 4)")
    n = np.linalg.norm(q, axis=-1)
    if np.any(np.abs(n - 1.0) > 1e-6):
        raise AssertionError("Quaternion must be unit norm")
    if np.allclose(q[..., :3], 0.0, atol=1e-6) and not np.allclose(q[..., 3], 1.0, atol=1e-6):
        raise AssertionError("Quaternion must be in [x, y, z, w] order")
