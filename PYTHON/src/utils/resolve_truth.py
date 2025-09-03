"""Python analogue of ``MATLAB/src/utils/resolve_truth.m``."""

from __future__ import annotations

from pathlib import Path
import os


def resolve_truth(preferred_path: str | None = None) -> str:
    """Return a resolved path to a truth file.

    Priority order matches ``resolve_truth_path``:
    - explicit ``preferred_path`` if provided and exists
    - environment ``TRUTH_FILE`` if set and exists
    - canonical ``DATA/Truth/STATE_X001.txt`` if exists
    - single ``STATE_*.txt`` in ``DATA/Truth`` if exactly one exists
    - otherwise, return empty string
    """

    # Robustly locate repo root: find 'PYTHON' dir then go up one
    here = Path(__file__).resolve()
    root = None
    for parent in here.parents:
        if parent.name == "PYTHON":
            root = parent.parent
            break
    if root is None:
        root = here.parents[2]
    truth_dir = root / "DATA" / "Truth"

    # Preferred explicit argument
    if preferred_path:
        p = Path(preferred_path)
        if p.is_file():
            print(f"Using TRUTH: {p}")
            return str(p)

    # Environment override
    env_path = os.environ.get("TRUTH_FILE")
    if env_path:
        p = Path(env_path)
        if p.is_file():
            print(f"Using TRUTH: {p}")
            return str(p)

    # Canonical path
    preferred = truth_dir / "STATE_X001.txt"
    if preferred.is_file():
        print(f"Using TRUTH: {preferred}")
        return str(preferred)

    # Single file fallback
    if truth_dir.is_dir():
        state_txts = sorted(truth_dir.glob("STATE_*.txt"))
        if len(state_txts) == 1 and state_txts[0].is_file():
            print(f"Using TRUTH: {state_txts[0]}")
            return str(state_txts[0])

    print("Truth file not found.")
    return ""


__all__ = ["resolve_truth"]
