"""Stub for MATLAB ``plot_state_grid`` equivalent."""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

def plot_state_grid(t: np.ndarray, pos: np.ndarray, vel: np.ndarray, acc: np.ndarray,
                    frame: str, *, visible: str = "off", save_dir: str | None = None,
                    run_id: str = "") -> None:
    """Placeholder plotting helper.

    Mirrors the MATLAB ``plot_state_grid`` function. The full implementation is
    pending; this stub accepts the same arguments so code can import it without
    failure.

    Parameters
    ----------
    t : ndarray
        Time vector [s].
    pos, vel, acc : ndarray
        N×3 arrays of position, velocity and acceleration in the specified frame.
    frame : str
        Coordinate frame label (e.g. ``"NED"``).
    visible : {'on','off'}, optional
        Show figure windows (default ``'off'``).
    save_dir : str, optional
        Directory to save plots.
    run_id : str, optional
        Identifier used in filenames.
    """
    _ = (t, pos, vel, acc, frame, visible, save_dir, run_id)
    # TODO: implement plotting similar to MATLAB version
    pass
