"""Utilities for locating the canonical truth file.

This mirrors ``MATLAB/src/utils/resolve_truth_path.m`` and returns the
preferred path for the truth file if it exists.
"""

from __future__ import annotations

from pathlib import Path


def resolve_truth_path() -> str | None:
    """Resolve the path to the truth file.

    Returns the canonical truth file path if found, otherwise ``None``.
    
    Searches for truth files in the repository root directory:
    - STATE_X001.txt (preferred)
    - STATE_X001_small.txt (fallback)
    """
    
    # Get repository root (two levels up from this file: src/utils -> src -> root)
    repo_root = Path(__file__).resolve().parents[2]
    
    # Try preferred truth file first
    preferred = repo_root / "STATE_X001.txt"
    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)
    
    # Try fallback truth file
    fallback = repo_root / "STATE_X001_small.txt"
    if fallback.is_file():
        print(f"Using TRUTH (small): {fallback}")
        return str(fallback)

    # Try other potential truth files
    for pattern in ["STATE_*.txt", "*STATE*.txt"]:
        matches = list(repo_root.glob(pattern))
        if matches:
            truth_file = matches[0]  # Use first match
            print(f"Using TRUTH (found): {truth_file}")
            return str(truth_file)
    
    print(f"No truth file found in {repo_root}")
    return None
