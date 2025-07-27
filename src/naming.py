"""Utility helpers to standardise output filenames.

These functions centralise how dataset tags and figure paths are
constructed so that both Python and MATLAB scripts produce
consistent results.
"""
from __future__ import annotations

from pathlib import Path


def build_tag(imu_file: str, gnss_file: str, method: str) -> str:
    """Return dataset tag ``IMU_Xnnn_GNSS_Xnnn_METHOD``."""
    imu_base = Path(imu_file).stem
    gnss_base = Path(gnss_file).stem
    return f"{imu_base}_{gnss_base}_{method}"


def prefix_filename(tag: str | None, filename: str) -> str:
    """Prepend ``tag`` to ``filename`` if provided."""
    return f"{tag}_{filename}" if tag else filename


def plot_path(
    results_dir: str | Path,
    tag: str,
    task: int,
    subtask: str,
    output_type: str,
    ext: str = ".pdf",
) -> Path:
    """Compose a plot filename following the repository convention."""
    results_dir = Path(results_dir)
    fname = f"{tag}_task{task}_{subtask}_{output_type}{ext}"
    return results_dir / fname
