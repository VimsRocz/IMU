"""Utilities for locating the canonical truth file.

This mirrors ``MATLAB/src/utils/resolve_truth_path.m``. It selects a
preferred path for the truth file and, if missing but an alternate exists,
copies the alternate to the preferred location. The function returns the
path to the truth file if one is found or copied, otherwise ``None``.
"""

from __future__ import annotations

from pathlib import Path
import shutil


def resolve_truth_path() -> str | None:
    """Resolve the path to the truth file.

    Returns the canonical truth file path, copying from an alternate if
    necessary. If neither file exists, ``None`` is returned.
    """

    preferred = Path("/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt")
    alternate = Path("/Users/vimalchawda/Desktop/IMU/STATE_X001.txt")

    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    if alternate.is_file():
        try:
            print(f"Truth missing at preferred path; copying {alternate} -> {preferred}")
            shutil.copy2(alternate, preferred)
            print(f"Using TRUTH: {preferred}")
            return str(preferred)
        except Exception as exc:  # pragma: no cover - best effort
            print(f"Copy to preferred failed ({exc}). Falling back to alternate.")
            return str(alternate)

    return None
