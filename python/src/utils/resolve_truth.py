"""Python analogue of ``MATLAB/src/utils/resolve_truth.m``."""

from __future__ import annotations

from pathlib import Path


def resolve_truth(preferred_path: str | None = None) -> str:
    """Return path to canonical truth file."""

    if not preferred_path:
        preferred_path = "/Users/vimalchawda/Desktop/IMU/STATE_X001.txt"
    preferred = Path(preferred_path)

    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    print("Truth file not found at preferred path.")
    return ""


__all__ = ["resolve_truth"]

