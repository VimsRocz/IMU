"""Utilities for locating the canonical truth file.

This mirrors ``MATLAB/src/utils/resolve_truth_path.m`` and returns the
preferred path for the truth file if it exists.
"""

from __future__ import annotations

from pathlib import Path
import os


def resolve_truth_path() -> str | None:
    """Resolve the path to the truth file.

    Priority order:
    1) Environment variable ``TRUTH_FILE`` if set and exists
    2) Canonical ``DATA/Truth/STATE_X001.txt`` if exists
    3) If exactly one ``STATE_*.txt`` exists under ``DATA/Truth``, use it
    4) Otherwise, return ``None``
    """

    root = Path(__file__).resolve().parents[2]
    truth_dir = root / "DATA" / "Truth"

    # 1) Explicit override via environment variable
    env_path = os.environ.get("TRUTH_FILE")
    if env_path:
        p = Path(env_path)
        if p.is_file():
            print(f"Using TRUTH: {p}")
            return str(p)

    # 2) Canonical preferred path
    preferred = truth_dir / "STATE_X001.txt"
    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    # 3) Single available STATE_*.txt in DATA/Truth
    if truth_dir.is_dir():
        state_txts = sorted(truth_dir.glob("STATE_*.txt"))
        if len(state_txts) == 1 and state_txts[0].is_file():
            print(f"Using TRUTH: {state_txts[0]}")
            return str(state_txts[0])

    # 4) None if not found
    return None
