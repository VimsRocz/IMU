#!/usr/bin/env python3
"""Command line interface to run all datasets using the TRIAD method.

This is an alias for ``run_triad_only.py`` for users accustomed to the
``*_cli.py`` naming pattern.  It simply forwards all command line
arguments to ``run_method_only.py --method TRIAD`` which runs Tasks 1--5
and, when truth logs are present, the Task 6 overlay and Task 7
evaluation.

Usage
-----
    python src/run_triad_only_cli.py [options]
"""
from pathlib import Path as _Path
import sys as _sys
_SRC = _Path(__file__).resolve().parent
if str(_SRC) not in _sys.path:
    _sys.path.insert(0, str(_SRC))
REPO_ROOT = _SRC.parents[2]

from run_method_only import main
import sys

if __name__ == "__main__":
    main(["--method", "TRIAD", *sys.argv[1:]])
