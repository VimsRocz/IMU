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
            "popup_figures": False,
            "save_png": True,
        },
        "zupt": {
            "acc_movstd_thresh": 0.15,
            "min_pre_lift_s": 5,
            "speed_thresh_mps": 0.30,
        },
    }
