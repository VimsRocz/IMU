"""Utility to load STATE_X ground truth files.

This mirrors ``read_state_file.m`` by returning the numeric
contents of the file while ignoring lines starting with ``#``.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np


def read_state_file(filename: str | Path) -> np.ndarray:
    """Return numeric array from a STATE_X file."""
    path = Path(filename)
    return np.loadtxt(path, comments="#")
