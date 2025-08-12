from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, Tuple, Any

import numpy as np


def save_task1_artifacts(out_dir: str | Path, tag: str, meta: Dict[str, Any], arrays: Dict[str, np.ndarray], gnss_columns: Iterable[str]) -> Tuple[Path, Path]:
    """Save Task 1 arrays to NPZ and metadata to JSON.

    Parameters
    ----------
    out_dir : path-like
        Directory where files will be stored.
    tag : str
        Common filename stem (e.g., ``IMU_X001_GNSS_X001_TRIAD``).
    meta : dict
        Metadata describing the dataset and method.
    arrays : dict
        Numeric arrays to persist.
    gnss_columns : iterable of str
        Column names from the GNSS file.
    Returns
    -------
    (npz_path, json_path) : tuple of :class:`Path`
        Paths of the written NPZ and JSON files.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    npz_path = out_dir / f"{tag}_task1_artifacts.npz"
    json_path = out_dir / f"{tag}_task1_results.json"

    np.savez(npz_path, **arrays)
    json_out = {"meta": meta, "gnss_columns": list(gnss_columns)}
    json_path.write_text(json.dumps(json_out, indent=2))
    print(f"Task 1: saved artifacts -> {npz_path.name}, {json_path.name}")
    return npz_path, json_path


def load_task1_artifacts(npz_path: str | Path) -> Dict[str, Any]:
    """Load Task 1 arrays and metadata from disk.

    Parameters
    ----------
    npz_path : path-like
        Path to ``*_task1_artifacts.npz`` file.

    Returns
    -------
    dict
        Combined dictionary with arrays, ``gnss_columns`` and ``meta``.
    """
    npz_path = Path(npz_path)
    data = {k: v for k, v in np.load(npz_path, allow_pickle=True).items()}
    json_path = npz_path.with_name(npz_path.name.replace("_artifacts.npz", "_results.json"))
    if json_path.exists():
        meta_json = json.loads(json_path.read_text())
        data["gnss_columns"] = meta_json.get("gnss_columns", [])
        data["meta"] = meta_json.get("meta", {})
    return data


__all__ = ["save_task1_artifacts", "load_task1_artifacts"]
