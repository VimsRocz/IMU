"""Stub for MATLAB ``ensure_dir``."""
from __future__ import annotations

import pathlib


def ensure_dir(path: str) -> None:
    """Create *path* if it does not exist."""
    pathlib.Path(path).mkdir(parents=True, exist_ok=True)
