"""Utility to shift time vector so that it starts at zero.

This mirrors the MATLAB ``zero_base_time`` helper.
"""
from __future__ import annotations
import numpy as np

def zero_base_time(t: np.ndarray) -> np.ndarray:
    """Return *t* as a float array starting at zero.

    Parameters
    ----------
    t : array_like
        Input time vector in seconds.

    Returns
    -------
    np.ndarray
        Time vector shifted so ``t[0] == 0``. Empty inputs are returned
        unchanged.
    """
    t = np.asarray(t, dtype=float).reshape(-1)
    if t.size == 0:
        return t
    return t - t[0]
