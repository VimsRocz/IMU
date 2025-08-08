"""Time-series interpolation helper matching the MATLAB version."""
from __future__ import annotations
import numpy as np

def interp_to(t_src: np.ndarray, y_src: np.ndarray, t_tgt: np.ndarray) -> np.ndarray:
    """Interpolate *y_src* sampled at *t_src* to the times *t_tgt*.

    Linear interpolation with extrapolation is used to mirror the MATLAB
    helper.
    """
    t_src = np.asarray(t_src, dtype=float).reshape(-1)
    t_tgt = np.asarray(t_tgt, dtype=float).reshape(-1)
    y_src = np.asarray(y_src, dtype=float)
    if y_src.ndim == 1:
        y_src = y_src.reshape(-1, 1)
    out = np.empty((t_tgt.size, y_src.shape[1]), dtype=float)
    for j in range(y_src.shape[1]):
        out[:, j] = np.interp(t_tgt, t_src, y_src[:, j], left=None, right=None)
    return out
