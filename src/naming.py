"""Utility functions for standardised file naming.

These helpers mirror the MATLAB `naming_utils` functions and are used
across the codebase to generate dataset-method tags, script names and
output filenames. The goal is to keep naming consistent between Python
and MATLAB implementations.
"""
from pathlib import Path
from typing import Union

def make_tag(dataset: str, gnss: str, method: str) -> str:
    """Return a dataset tag like ``IMU_X001_GNSS_X001_TRIAD``."""
    dname = Path(dataset).stem
    gname = Path(gnss).stem
    return f"{dname}_{gname}_{method}"

def script_name(dataset: str, method: str, task: int, ext: str = "py") -> str:
    """Return a script filename for a given dataset, method and task."""
    dname = Path(dataset).stem
    return f"{dname}_{method}_task{task}.{ext}"

def output_dir(task: int, dataset: str, gnss: str, method: str, base_dir: Union[str, Path] = "results") -> Path:
    """Return the directory path for storing outputs for a given task."""
    tag = make_tag(dataset, gnss, method)
    base = Path(base_dir)
    return base / f"task{task}" / tag

def plot_filename(dataset: str, gnss: str, method: str, task: int, subtask: str, out_type: str, ext: str = "pdf") -> str:
    """Return a plot filename following the standard convention."""
    tag = make_tag(dataset, gnss, method)
    return f"{tag}_task{task}_{subtask}_{out_type}.{ext}"

def plot_path(base_dir: Union[str, Path], tag: str, task: int, subtask: str, out_type: str, ext: str = "pdf") -> Path:
    """Return the path to a plot under ``base_dir`` using the standard naming."""
    base_dir = Path(base_dir)
    return base_dir / f"{tag}_task{task}_{subtask}_{out_type}.{ext}"
