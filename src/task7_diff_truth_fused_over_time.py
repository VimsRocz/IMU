"""Task 7.5: Plot truth minus fused differences over time (stub).

This stub mirrors ``MATLAB/evaluate_filter_results.m`` Subtask 7.5.
It will compute ``truth - fused`` position and velocity in the NED frame
and plot the components over time. Figures will be saved in the
``results/task7/<run_id>/`` directory.

Todo
----
Implement the data loading, difference computation and plotting.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np


def plot_truth_fused_diff(
    time: Iterable[float],
    fused_pos_ned: np.ndarray,
    truth_pos_ned: np.ndarray,
    fused_vel_ned: np.ndarray,
    truth_vel_ned: np.ndarray,
    run_id: str,
    output_dir: Path,
) -> None:
    """Placeholder for the MATLAB Subtask 7.5 equivalent."""
    raise NotImplementedError

