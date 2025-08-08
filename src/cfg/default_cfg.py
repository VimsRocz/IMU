"""Default configuration stub mirroring MATLAB ``cfg.default_cfg``."""
from __future__ import annotations

from .. import project_paths  # type: ignore


def default_cfg() -> dict:
    """Return default configuration dictionary."""
    paths = project_paths.project_paths() if hasattr(project_paths, "project_paths") else {}
    return {
        "dataset_id": "",
        "method": "",
        "imu_file": "",
        "gnss_file": "",
        "truth_file": "",
        "paths": paths,
        "plots": {
            "popup_figures": True,
            "save_pdf": True,
            "save_png": True,
        },
    }
