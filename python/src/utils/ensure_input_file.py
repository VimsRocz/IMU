"""Locate a dataset file within standard repository folders.

This mirrors the MATLAB ``get_data_file`` utility and searches the repo
root as well as ``DATA/IMU``, ``DATA/GNSS`` and ``DATA/TRUTH``.

Usage
-----
    ensure_input_file('IMU_X002.dat')
"""
from __future__ import annotations

from pathlib import Path


def _search_roots() -> list[Path]:
    """Return candidate directories for input data."""
    repo = Path(__file__).resolve().parents[2]
    data = repo / "DATA"
    return [repo, data / "IMU", data / "GNSS", data / "TRUTH"]


def ensure_input_file(path_like: str) -> Path:
    """Resolve *path_like* to an existing dataset file."""
    p = Path(path_like)
    if p.is_absolute() and p.exists():
        return p
    if p.exists():
        return p.resolve()
    for r in _search_roots():
        candidate = r / p.name
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(f"Dataset not found: {path_like}")
