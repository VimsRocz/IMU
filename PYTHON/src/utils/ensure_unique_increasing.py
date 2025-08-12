"""Time utilities mirroring MATLAB ``ensure_unique_increasing``."""
from __future__ import annotations

import numpy as np


def ensure_unique_increasing(label: str, t: np.ndarray, Y: np.ndarray | None = None):
    """Return strictly increasing unique times with optional data alignment.

    Parameters
    ----------
    label : str
        Identifier used in log messages.
    t : np.ndarray
        1-D array of time stamps.
    Y : np.ndarray, optional
        Array of samples aligned with ``t``.
    """
    if Y is None:
        Y = None
    t = np.asarray(t).reshape(-1)
    mask = np.isfinite(t)
    if not np.all(mask):
        t = t[mask]
        if Y is not None:
            Y = Y[mask]

    order = np.argsort(t)
    t = t[order]
    if Y is not None:
        Y = Y[order]

    uniq, idx = np.unique(t, return_index=True)
    if Y is not None:
        Y = Y[idx]

    info = {
        "removed_nonfinite": int((~mask).sum()),
        "removed_duplicates": int(len(t) - len(uniq)),
        "monotonic": bool(np.all(np.diff(uniq) > 0)),
    }
    if not info["monotonic"]:
        step_mask = np.hstack(([True], np.diff(uniq) > 0))
        uniq = uniq[step_mask]
        if Y is not None:
            Y = Y[step_mask]

    if info["removed_nonfinite"] or info["removed_duplicates"] or not info["monotonic"]:
        print(
            f"[time-fix] {label}: drop nonfinite={info['removed_nonfinite']}, "
            f"dups={info['removed_duplicates']}, monotonic={int(info['monotonic'])}"
        )
    return uniq, Y, info
