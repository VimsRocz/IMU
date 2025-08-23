from __future__ import annotations

from pathlib import Path
from datetime import datetime
import matplotlib.pyplot as plt
from utils import save_png_and_mat
from utils.matlab_fig_export import save_matlab_fig


def ensure_output_dirs(run_name: str, task_name: str) -> Path:
    """Ensure output directory exists for a given run and task.

    Parameters
    ----------
    run_name : str
        Name of the run (e.g. "X001").
    task_name : str
        Name of task directory (e.g. "Task_1").

    Returns
    -------
    Path
        Path to the created directory.
    """
    out_root = Path(__file__).resolve().parents[2] / "OUTPUTS"
    out_dir = out_root / run_name / task_name
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def save_plot(fig: plt.Figure, out_dir: Path, stem: str, ext: str = "png") -> Path:
    """Save a Matplotlib figure using timestamped and latest filenames.

    Parameters
    ----------
    fig : plt.Figure
        Figure to save.
    out_dir : Path
        Target directory.
    stem : str
        Base filename without extension.
    ext : str, optional
        File extension, by default "png".

    Returns
    -------
    Path
        Path to the timestamped file.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    # Prefer native MATLAB .fig; else fall back to PNG+MAT
    timestamped = out_dir / f"{stem}_{ts}"
    latest = out_dir / f"{stem}_latest"
    if save_matlab_fig(fig, str(timestamped)) is None:
        save_png_and_mat(fig, str(timestamped))
    else:
        try:
            fig.savefig(str(timestamped) + ".png", dpi=200, bbox_inches='tight')
        except Exception:
            pass
    if save_matlab_fig(fig, str(latest)) is None:
        save_png_and_mat(fig, str(latest))
    else:
        try:
            fig.savefig(str(latest) + ".png", dpi=200, bbox_inches='tight')
        except Exception:
            pass
    return Path(str(timestamped) + ".fig")
