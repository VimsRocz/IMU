"""Utilities for locating the canonical truth file.

This mirrors ``MATLAB/src/utils/resolve_truth_path.m`` and returns the
preferred path for the truth file if it exists.
"""

from __future__ import annotations

from pathlib import Path


def resolve_truth_path() -> str | None:
    """Resolve the path to the truth file.

    Returns the canonical truth file path if found, otherwise ``None``.
    """

    preferred = Path(__file__).resolve().parents[2] / "DATA" / "Truth" / "STATE_X001.txt"

    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    return None
