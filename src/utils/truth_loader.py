"""Placeholder for the MATLAB ``TruthLoader`` utility."""
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
import numpy as np
import pandas as pd


@dataclass
class Truth:
    """Container for truth data."""
    t_posix: np.ndarray
    t0: np.ndarray
    n: int
    pos_ecef: Optional[np.ndarray] = None
    vel_ecef: Optional[np.ndarray] = None
    att_quat_wxyz: Optional[np.ndarray] = None
    notes: Optional[List[str]] = None


def truth_loader(path: str) -> Truth:
    """Read a whitespace-delimited truth file.

    This minimal implementation parses the first column as time and assumes
    seconds. It is a stub to maintain parity with the MATLAB version.
    """
    df = pd.read_csv(path, comment="#", delim_whitespace=True)
    t = df.iloc[:, 0].to_numpy(dtype=float)
    t0 = t - t[0]
    return Truth(t_posix=t, t0=t0, n=t.size, notes=["python-stub"])
