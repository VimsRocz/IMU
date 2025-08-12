"""Basic validation helpers used across pipeline tasks."""

from __future__ import annotations

import numpy as np


def ensure_monotonic(t, logger=None, dt_hint=None):
    """Return a strictly increasing time vector, repairing if needed."""
    t = np.asarray(t, dtype=float)
    if np.all(np.diff(t) > 0):
        return t, None, 0

    dt = np.diff(t)
    pos = dt[dt > 0]
    if pos.size == 0 and (not dt_hint or dt_hint <= 0):
        raise ValueError(
            "time vector cannot be repaired (no positive dt and no dt_hint)."
        )
    dt_med = float(dt_hint) if (dt_hint and dt_hint > 0) else float(np.median(pos))

    t_fix = t.copy()
    violations = 0
    for i in range(1, t_fix.size):
        if t_fix[i] <= t_fix[i - 1]:
            t_fix[i] = t_fix[i - 1] + dt_med
            violations += 1

    if logger:
        logger.warning(
            f"[IMU time repair] fixed {violations} non-monotonic steps using dt={dt_med:.6f}s "
            f"(orig {t[0]:.6f}->{t[-1]:.6f}, fixed {t_fix[0]:.6f}->{t_fix[-1]:.6f})"
        )
    return t_fix, dt_med, violations


def assert_monotonic(t: np.ndarray) -> None:
    """Raise ``ValueError`` if ``t`` is not strictly increasing."""
    if np.any(np.diff(t) <= 0):
        raise ValueError("time vector is not strictly increasing")


def assert_rate_stability(t: np.ndarray, hz_expected: float | None = None, tol_ppm: float = 1000) -> float:
    """Validate sample period stability and return estimated rate."""
    if t.size < 2:
        return float("nan")
    dt = np.diff(t)
    med = float(np.median(dt))
    if hz_expected is not None:
        hz = 1.0 / med
        ppm = abs(hz - hz_expected) / hz_expected * 1e6
        if ppm > tol_ppm:
            raise ValueError("sample rate deviates beyond tolerance")
    return 1.0 / med


def assert_no_nans(name: str, arr: np.ndarray) -> None:
    """Error if ``arr`` contains NaN values."""
    if np.isnan(arr).any():
        raise ValueError(f"{name} contains NaN")


def assert_shape(name: str, arr: np.ndarray, shape) -> None:
    """Ensure ``arr`` conforms to ``shape`` allowing ``-1`` as wildcard."""
    expected = tuple(shape)
    actual = arr.shape
    if len(expected) != len(actual):
        raise ValueError(f"{name} has wrong rank: {actual} vs {expected}")
    for a, e in zip(actual, expected):
        if e != -1 and a != e:
            raise ValueError(f"{name} has shape {actual}, expected {expected}")


__all__ = [
    "ensure_monotonic",
    "assert_monotonic",
    "assert_rate_stability",
    "assert_no_nans",
    "assert_shape",
]
