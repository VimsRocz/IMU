"""Simple ternary helper mirroring MATLAB ``ternary``."""
from __future__ import annotations


def ternary(cond: bool, a, b):
    """Return *a* if *cond* else *b*."""
    return a if cond else b
