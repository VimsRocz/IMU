"""Time utility helpers used throughout the project.

This module mirrors a small subset of the MATLAB helper functions.  In
particular it provides a simple routine for estimating the time offset
between two synchronised sequences via cross-correlation.  The
implementation matches the behaviour of ``compute_time_shift.m`` in the
MATLAB codebase.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .ensure_unique_increasing import ensure_unique_increasing

__all__ = ["ensure_unique_increasing", "compute_time_shift"]


def compute_time_shift(
    est_series: np.ndarray, truth_series: np.ndarray, dt: float
) -> Tuple[int, float]:
    """Estimate sample lag and time shift between two series.

    Parameters
    ----------
    est_series, truth_series:
        One-dimensional arrays sampled at the same rate.  The *truth* series
        is considered to be delayed relative to the *est* series.  The series
        are demeaned prior to cross-correlation to avoid bias from offsets.
    dt:
        Sample interval of ``est_series`` in seconds.

    Returns
    -------
    lag:
        Integer sample offset that maximises the cross-correlation
        (truth relative to estimate).
    t_shift:
        Corresponding time shift in seconds such that ``t_shift = lag * dt``.

    Notes
    -----
    Positive ``lag`` means the truth data starts later than the estimate.  To
    align the truth timestamps to the estimate, subtract ``t_shift`` from the
    truth time vector.
    """

    est = np.asarray(est_series, dtype=float).ravel()
    truth = np.asarray(truth_series, dtype=float).ravel()
    if est.size != truth.size:
        raise ValueError("est_series and truth_series must have the same length")
    if est.size == 0:
        raise ValueError("Input series must not be empty")

    est = est - np.mean(est)
    truth = truth - np.mean(truth)

    corr = np.correlate(est, truth, mode="full")
    lag = int(np.argmax(corr) - (est.size - 1))
    t_shift = lag * dt
    return lag, t_shift
