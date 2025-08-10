"""Stub for MATLAB ``require_files``."""
from __future__ import annotations

import pathlib
from typing import Iterable


def require_files(paths: Iterable[str]) -> None:
    """Assert that all *paths* exist as files."""
    for p in paths:
        if not pathlib.Path(p).is_file():
            raise FileNotFoundError(f"Required file not found: {p}")
