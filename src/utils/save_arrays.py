"""Helpers to serialise numerical arrays to disk."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import savemat


def save_npz(base: str | Path, **arrays: Any) -> str:
    """Save arrays to ``base`` with ``.npz`` extension and return the path."""
    path = Path(base).with_suffix(".npz")
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)
    return str(path)


def save_mat(base: str | Path, **arrays: Any) -> str:
    """Save arrays to ``base`` with ``.mat`` extension and return the path."""
    path = Path(base).with_suffix(".mat")
    path.parent.mkdir(parents=True, exist_ok=True)
    savemat(path, arrays)
    return str(path)


def save_json(base: str | Path, obj: Any) -> str:
    """Serialise ``obj`` as JSON to ``base`` with ``.json`` extension."""
    path = Path(base).with_suffix(".json")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2, sort_keys=True)
    return str(path)


__all__ = ["save_npz", "save_mat", "save_json"]
