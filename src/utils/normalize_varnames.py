"""Utility mirroring MATLAB ``normalize_varnames``.

The implementation is intentionally lightweight and primarily exists to
maintain a parallel API between the Python and MATLAB code bases.
"""
from __future__ import annotations
from typing import Sequence, List


def normalize_varnames(names: Sequence[str]) -> List[str]:
    """Return lower-case names stripped of invalid characters.

    Parameters
    ----------
    names : sequence of str
        Original column names.

    Returns
    -------
    list of str
        Normalised names consisting of lowercase letters, digits and
        underscores only.
    """
    out: List[str] = []
    for n in names:
        n = n.lower()
        n = ''.join(ch for ch in n if ch.isalnum() or ch == '_')
        out.append(n)
    return out
