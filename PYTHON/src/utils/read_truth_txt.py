from __future__ import annotations

from pathlib import Path
import numpy as np


def read_truth_txt(path: str | Path):
    """Return ``t_truth``, ``pos_ecef``, ``vel_ecef`` and ``quat_wxyz`` arrays.

    The truth files have columns::
        count  time  X_ECEF_m  Y_ECEF_m  Z_ECEF_m  VX_ECEF_mps  VY_ECEF_mps
        VZ_ECEF_mps  q0  q1  q2  q3

    ``vel`` and ``quat`` columns are optional; missing series are returned as
    zeros with a warning.
    """

    p = Path(path)
    arr = np.loadtxt(p, comments="#")
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.size == 0:
        return (
            np.array([]),
            np.zeros((0, 3)),
            np.zeros((0, 3)),
            np.zeros((0, 4)),
        )
    if arr.shape[1] < 5:
        raise ValueError(f"Truth file {p} has insufficient columns: {arr.shape[1]}")

    t_truth = arr[:, 1].astype(float)
    pos_ecef = arr[:, 2:5].astype(float)

    if arr.shape[1] >= 8:
        vel_ecef = arr[:, 5:8].astype(float)
    else:
        vel_ecef = np.zeros_like(pos_ecef)
        print(f"[Task6][WARN] {p} lacks velocity columns; using zeros")

    if arr.shape[1] >= 12:
        quat_wxyz = arr[:, 8:12].astype(float)
    else:
        quat_wxyz = np.zeros((arr.shape[0], 4))
        print(f"[Task6][WARN] {p} lacks quaternion columns; using zeros")

    return t_truth, pos_ecef, vel_ecef, quat_wxyz


__all__ = ["read_truth_txt"]
