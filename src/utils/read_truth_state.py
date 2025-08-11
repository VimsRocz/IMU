"""Stub mirroring ``MATLAB/src/utils/read_truth_state.m``."""
from __future__ import annotations

from pathlib import Path
import numpy as np

from ..read_state_file import read_state_file


def read_truth_state(truth_path: str | Path) -> np.ndarray:
    """Return time vector from a truth state file.

    Parameters
    ----------
    truth_path : str or Path
        Path to ``STATE_X001.txt``.

    Returns
    -------
    numpy.ndarray
        1-D array of time values in seconds from column 1 of the file.
    """

    data = read_state_file(truth_path)
    if data.size == 0:
        return np.array([])
    # Column 0 holds the sample count while column 1 is time in seconds.
    return np.asarray(data[:, 1]).reshape(-1)


__all__ = ["read_truth_state"]

