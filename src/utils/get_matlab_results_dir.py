"""MATLAB parity helper.

This module mirrors ``get_matlab_results_dir.m`` in MATLAB. It is not used
in the Python pipeline but provides structural symmetry between languages.
"""
from pathlib import Path


def get_matlab_results_dir() -> Path:
    """Return the MATLAB-specific results directory.

    The Python pipeline writes to ``results/`` while MATLAB writes to
    ``MATLAB/results``. This helper exposes the latter for completeness.
    """
    return Path(__file__).resolve().parents[2] / "MATLAB" / "results"

