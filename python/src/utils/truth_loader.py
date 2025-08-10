"""Stub for MATLAB ``TruthLoader.m``.

This placeholder exists for cross-language parity. The MATLAB version
implements robust ground truth loading; a Python equivalent has not yet
been implemented.
"""
from __future__ import annotations
from typing import Any, Dict


def truth_loader(truth_path: str, opts: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """Placeholder for MATLAB ``TruthLoader``.

    Parameters
    ----------
    truth_path : str
        Path to truth file or directory.
    opts : dict, optional
        Reserved for future options.
    """
    raise NotImplementedError("truth_loader is only implemented in MATLAB")


__all__ = ["truth_loader"]
