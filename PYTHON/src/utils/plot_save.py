from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Dict, List

_saved: Dict[str, List[str]] = defaultdict(list)


def save_plot(fig, results_dir: str | Path, run_id: str, task_label: str, plot_label: str, ext: str = "png", **kwargs) -> Path:
    """Save *fig* to ``results_dir`` with a standardised filename.

    Parameters
    ----------
    fig : matplotlib.figure.Figure
        Figure object to save.
    results_dir : str or Path
        Base directory where plots are stored.
    run_id : str
        Identifier for the current run (e.g. ``IMU_X002_GNSS_X002_TRIAD``).
    task_label : str
        Task label such as ``"task1"`` or ``"task6"``.
    plot_label : str
        Short descriptor for the specific plot.
    ext : str, optional
        File extension, defaults to ``"png"``.
    kwargs : dict
        Extra arguments forwarded to :meth:`matplotlib.figure.Figure.savefig`.
    """
    results_dir = Path(results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    out_path = results_dir / f"{run_id}_{task_label}_{plot_label}.{ext}"
    fig.savefig(out_path, **kwargs)
    print(f"[SAVE] {out_path}")
    _saved[task_label].append(out_path.name)
    return out_path


def task_summary(task_label: str) -> None:
    """Print a summary of saved plots for *task_label*."""
    files = _saved.get(task_label, [])
    n = task_label.replace("task", "")
    print(f"[TASK {n}] Plots saved to results/: {files}")
