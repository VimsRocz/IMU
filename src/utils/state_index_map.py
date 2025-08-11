"""Utility to build index maps for state vectors.

This mirrors the MATLAB ``state_index_map`` helper.

Usage:
    layout = OrderedDict([('pos',3),('vel',3),('att',3)])
    idx = state_index_map(layout)
"""

from typing import Dict, List


def state_index_map(layout: Dict[str, int]) -> Dict[str, List[int]]:
    """Create an index map for state vector blocks.

    Parameters
    ----------
    layout : dict
        Ordered mapping of block names to their sizes.

    Returns
    -------
    dict
        Mapping of block names to index ranges (1-based), including ``N``.
    """
    names = list(layout.keys())
    s = 1
    idx = {}
    for name in names:
        n = int(layout[name])
        idx[name] = list(range(s, s + n))
        s += n
    idx['N'] = s - 1
    return idx
