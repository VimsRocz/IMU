"""Python analogue of ``MATLAB/src/utils/resolve_truth.m``."""

from __future__ import annotations

from pathlib import Path
import shutil


def resolve_truth(preferred_path: str | None = None) -> str:
    """Return path to canonical truth file, copying from fallback if needed."""

    if not preferred_path:
        preferred_path = "/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt"
    preferred = Path(preferred_path)
    fallback = Path("/Users/vimalchawda/Desktop/IMU/STATE_X001.txt")

    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    if fallback.is_file():
        try:
            print(f"Truth missing at preferred path; copying {fallback} -> {preferred}")
            shutil.copy2(fallback, preferred)
            print(f"Using TRUTH: {preferred}")
            return str(preferred)
        except Exception as exc:  # pragma: no cover
            print(f"Failed to copy truth file: {exc}")

    print("Truth file not found at preferred or fallback paths.")
    return ""


__all__ = ["resolve_truth"]

