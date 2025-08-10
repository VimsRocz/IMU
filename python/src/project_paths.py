"""Utility to discover repository paths (stub for MATLAB project_paths)."""
from __future__ import annotations

import pathlib


def project_paths() -> dict:
    """Return a dictionary with standard project paths.

    This is a lightweight Python counterpart to the MATLAB ``project_paths``
    helper.  Paths are resolved relative to this file.
    """
    here = pathlib.Path(__file__).resolve().parent
    root = here.parent
    return {
        "root": str(root),
        "results": str(root / "results"),
        "src_utils": str(root / "src" / "utils"),
        "matlab_utils": str(root / "MATLAB" / "utils"),
    }
