"""Utilities for locating the canonical truth file.

This mirrors ``MATLAB/src/utils/resolve_truth_path.m`` and returns the
preferred path for the truth file if it exists.
"""

from __future__ import annotations

from pathlib import Path
import os


def resolve_truth_path(dataset: str | None = None) -> str | None:
    """Resolve the path to the truth file.

    Priority order:
    1) Environment variable ``TRUTH_FILE`` if set and exists
    2) If ``dataset`` is provided, ``DATA/Truth/STATE_{dataset}.txt`` if exists
    3) Canonical ``DATA/Truth/STATE_X001.txt`` if exists
    4) If exactly one ``STATE_*.txt`` exists under ``DATA/Truth``, use it
    5) Otherwise, return ``None``
    """

    # Robustly locate repo root: find 'PYTHON' dir then go up one
    here = Path(__file__).resolve()
    root = None
    for parent in here.parents:
        if parent.name == "PYTHON":
            root = parent.parent
            break
    if root is None:
        # Fallback to previous heuristic
        root = here.parents[2]
    truth_dir = root / "DATA" / "Truth"

    # 1) Explicit override via environment variable
    env_path = os.environ.get("TRUTH_FILE")
    if env_path:
        p = Path(env_path)
        if p.is_file():
            print(f"Using TRUTH: {p}")
            return str(p)

    # 2) Dataset-specific path if requested
    if dataset:
        ds_name = f"STATE_{dataset}.txt"
        ds_path = truth_dir / ds_name
        if ds_path.is_file():
            print(f"Using TRUTH: {ds_path}")
            return str(ds_path)

    # 3) Canonical preferred path
    preferred = truth_dir / "STATE_X001.txt"
    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    # 4) Single available STATE_*.txt in DATA/Truth
    if truth_dir.is_dir():
        state_txts = sorted(truth_dir.glob("STATE_*.txt"))
        if len(state_txts) == 1 and state_txts[0].is_file():
            print(f"Using TRUTH: {state_txts[0]}")
            return str(state_txts[0])

    # 5) None if not found
    return None
