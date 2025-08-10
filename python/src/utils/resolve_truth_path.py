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
    root = Path(__file__).resolve().parents[2]
    candidates = [
        root / "DATA" / "TRUTH" / "STATE_X001.txt",
        root / "DATA" / "TRUTH" / "STATE_X001_small.txt",
    ]
    for c in candidates:
        if c.is_file():
            print(f"Using TRUTH: {c}")
            return str(c)
    matches = sorted((root / "DATA" / "TRUTH").glob("STATE_*.txt"))
    if matches:
        print(f"Using TRUTH: {matches[0]}")
        return str(matches[0])
    return None
