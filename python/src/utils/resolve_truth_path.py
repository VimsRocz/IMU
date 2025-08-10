"""Resolve paths to truth dataset files.

Searches DATA/TRUTH then repository root for a given file name.
"""

from __future__ import annotations

from pathlib import Path


def _truth_candidates(name: str) -> Path | None:
    repo = Path(__file__).resolve().parents[3]
    for d in [repo / "DATA" / "TRUTH", repo]:
        p = d / name
        if p.exists():
            return p
    return None


def resolve_truth_path(truth_path: str) -> Path:
    """Return absolute path to *truth_path*.

    Parameters
    ----------
    truth_path : str
        Absolute path, relative path, or bare filename for a truth file.
    """
    p = Path(truth_path)
    cand = _truth_candidates(p.name)
    return cand if cand else p.resolve()
