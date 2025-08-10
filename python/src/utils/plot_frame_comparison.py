"""Plot comparison helper (MATLAB counterpart).

This module mirrors the MATLAB function ``plot_frame_comparison`` but is not
implemented in Python yet.

TODO: Implement plotting logic to maintain parity with MATLAB version.
"""

from typing import Sequence
import numpy as np


def plot_frame_comparison(
    t: np.ndarray,
    data_sets: Sequence[np.ndarray],
    labels: Sequence[str],
    frame_name: str,
    out_prefix: str,
) -> None:
    """Placeholder for MATLAB ``plot_frame_comparison``.

    Parameters
    ----------
    t : np.ndarray
        Time vector.
    data_sets : Sequence[np.ndarray]
        Sequence of arrays with shape (N, 3) or (3, N).
    labels : Sequence[str]
        Legend labels for each dataset.
    frame_name : str
        Name of the frame (e.g. ``"NED"``).
    out_prefix : str
        Output path prefix.
    """
    raise NotImplementedError("Python version of plot_frame_comparison is pending")
