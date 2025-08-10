"""Utility to shift time vector so that it starts at zero.

This mirrors the MATLAB ``zero_base_time`` helper.
"""
from __future__ import annotations
import numpy as np

def zero_base_time(t: np.ndarray) -> np.ndarray:
    """Return *t* as a float array starting at zero.

    Handles :class:`numpy.datetime64` inputs and leading ``NaN`` values.
    Raises ``ValueError`` if the input is empty or all ``NaN``.
    """
    arr = np.asarray(t)
    if arr.size == 0:
        raise ValueError("zero_base_time: empty input")

    if np.issubdtype(arr.dtype, np.datetime64):
        arr = arr.astype("datetime64[s]").astype(float)
    else:
        arr = arr.astype(float)

    mask = ~np.isnan(arr)
    if not np.any(mask):
        raise ValueError("zero_base_time: all NaN input")

    arr = arr[mask]
    return arr - arr[0]
