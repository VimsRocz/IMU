from __future__ import annotations

from pathlib import Path
import numpy as np


def read_truth_txt(path: str | Path):
    """Return ``t_truth``, ``pos_ecef`` and ``vel_ecef`` arrays from STATE_X text file.

    The truth files have columns::
        count  time  X_ECEF_m  Y_ECEF_m  Z_ECEF_m  VX_ECEF_mps  VY_ECEF_mps  VZ_ECEF_mps ...
    ``vel`` columns are optional; if missing, zeros are returned with a warning.
    """
    p = Path(path)
    arr = np.loadtxt(p, comments="#")
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.size == 0:
        return np.array([]), np.zeros((0, 3)), np.zeros((0, 3))
    if arr.shape[1] < 5:
        raise ValueError(f"Truth file {p} has insufficient columns: {arr.shape[1]}")
    t_truth = arr[:, 1].astype(float)
    pos_ecef = arr[:, 2:5].astype(float)
    if arr.shape[1] >= 8:
        vel_ecef = arr[:, 5:8].astype(float)
    else:
        vel_ecef = np.zeros_like(pos_ecef)
        print(f"[Task6][WARN] {p} lacks velocity columns; using zeros")
    return t_truth, pos_ecef, vel_ecef


__all__ = ["read_truth_txt"]
