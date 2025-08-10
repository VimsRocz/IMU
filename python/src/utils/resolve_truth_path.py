"""Resolve a truth file path relative to standard repository locations."""
from __future__ import annotations

from pathlib import Path


def resolve_truth_path(truth_path: str) -> Path:
    """Return an absolute path to *truth_path*.

    If the provided path does not exist, the file name is searched within
    ``DATA/TRUTH`` under the repository root.
    """
    repo = Path(__file__).resolve().parents[2]
    candidate = repo / "DATA" / "TRUTH" / Path(truth_path).name
    if candidate.exists():
        return candidate
    return Path(truth_path).resolve()
