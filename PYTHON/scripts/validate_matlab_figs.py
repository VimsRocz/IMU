#!/usr/bin/env python3
"""Validate that MATLAB .fig files open in MATLAB (best-effort).

This script scans the ``results/`` directory for ``*.fig`` files, then uses
the MATLAB Engine (if available) to attempt to open and close each file.
It prints a summary report.

Usage:
  python scripts/validate_matlab_figs.py [results_dir]
"""
from __future__ import annotations

import sys
from pathlib import Path

from src.utils.matlab_fig_export import validate_fig_openable


def main(argv: list[str]) -> int:
    root = Path(argv[1]) if len(argv) > 1 else Path('results')
    if not root.exists():
        print(f"No such directory: {root}")
        return 2
    figs = sorted(root.rglob('*.fig'))
    if not figs:
        print(f"No .fig files found under {root}")
        return 0
    ok = 0
    for fp in figs:
        if validate_fig_openable(fp):
            ok += 1
    print(f"Validated {ok}/{len(figs)} .fig files (engine required)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))

