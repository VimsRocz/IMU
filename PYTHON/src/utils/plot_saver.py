from __future__ import annotations

import os
from typing import Any, Dict


def ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)


def save_png_and_mat(fig, basepath: str, arrays: Dict[str, Any] | None = None) -> None:
    """
    Save a Matplotlib or Plotly figure to PNG and (optionally) a .mat with the underlying data.

    - fig: Matplotlib Figure or Plotly Figure (both expose save APIs)
    - basepath: path without extension, e.g., ".../IMU_X002_GNSS_X002_DAVENPORT_task3_errors"
    - arrays: dict of numpy arrays to store inside the .mat (keys become variable names)
    """
    png = f"{basepath}.png"
    # Try Matplotlib first; fall back to Plotly
    try:
        fig.savefig(png, dpi=150, bbox_inches="tight")  # Matplotlib
    except Exception:
        # Plotly
        try:
            fig.write_image(png, scale=2)
        except Exception as e:
            print(f"[WARN] Could not save PNG {png}: {e}")
            return
    print(f"[SAVE] {png}")

    if arrays:
        try:
            from scipy.io import savemat
            savemat(f"{basepath}.mat", arrays, do_compression=True)
            print(f"[MAT ] {basepath}.mat keys={list(arrays.keys())}")
        except Exception as e:
            print(f"[WARN] Could not save MAT for {basepath}: {e}")

