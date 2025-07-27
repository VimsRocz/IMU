"""Utility functions for consistent file naming across tasks."""

from pathlib import Path


def make_tag(dataset: str, gnss: str, method: str) -> str:
    """Return ``IMU_X001_GNSS_X001_TRIAD`` style tag."""
    dataset_id = Path(dataset).stem
    gnss_id = Path(gnss).stem
    return f"{dataset_id}_{gnss_id}_{method}"


def script_name(dataset: str, method: str, task: int, ext: str = "py") -> str:
    """Return ``IMU_X001_TRIAD_task1.py`` for the given parameters."""
    dataset_id = Path(dataset).stem
    return f"{dataset_id}_{method}_task{task}.{ext}"


def output_dir(
    task: int, dataset: str, gnss: str, method: str, base: Path | str = "results"
) -> Path:
    """Return results directory path for the specific task and dataset."""
    tag = make_tag(dataset, gnss, method)
    return Path(base) / f"task{task}" / tag


def plot_filename(
    dataset: str,
    gnss: str,
    method: str,
    task: int,
    subtask: str,
    output_type: str,
    ext: str = "pdf",
) -> str:
    """Construct a standardised plot filename."""
    tag = make_tag(dataset, gnss, method)
    return f"{tag}_task{task}_{subtask}_{output_type}.{ext}"
