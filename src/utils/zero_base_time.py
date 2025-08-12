"""Utility to shift time vector so that it starts at zero.

This mirrors the MATLAB ``zero_base_time`` helper.
"""
from __future__ import annotations
import numpy as np

def zero_base_time(t: np.ndarray) -> np.ndarray:
    """Return *t* as a float array starting at zero.

    This mirrors the more robust MATLAB implementation and accepts
    ``datetime64`` inputs. Leading non-finite values are ignored and an
    error is raised if no finite entries remain.
    """
    t = np.asarray(t)
    if t.size == 0:
        raise ValueError("zero_base_time: empty")
    if np.issubdtype(t.dtype, np.datetime64):
        t = t.astype("datetime64[s]").astype(float)
    t = t.astype(float).reshape(-1)
    mask = np.isfinite(t)
    if not np.any(mask):
        raise ValueError("zero_base_time: no finite times")
    first = np.flatnonzero(mask)[0]
    t = t[first:]
    return t - t[0]
