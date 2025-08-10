"""Stub for MATLAB ``plot_state_grid_overlay`` utility."""
from __future__ import annotations

import numpy as np


def plot_state_grid_overlay(t_ref: np.ndarray, fused: dict, truth: dict,
                            frame_label: str, **kwargs) -> None:
    """Placeholder that mirrors MATLAB plotting utility.

    Parameters
    ----------
    t_ref : np.ndarray
        Reference time vector for plotting.
    fused : dict
        Dictionary with keys ``t``, ``pos``, ``vel`` and ``acc``.
    truth : dict
        Dictionary with keys ``t``, ``pos``, ``vel`` and ``acc``.
    frame_label : str
        Name of the reference frame (e.g. ``'NED'``).
    """
    raise NotImplementedError(
        "plot_state_grid_overlay is MATLAB-only; this is a placeholder"
    )
