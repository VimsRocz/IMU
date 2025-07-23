#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method.

This script forwards any additional command line arguments to
``run_all_methods.py`` with ``--methods TRIAD`` so that the behaviour
matches ``run_all_methods.py`` when the TRIAD method is selected.

Usage
-----
    python src/run_triad_only.py [options]
"""

from run_all_methods import main
import sys

if __name__ == "__main__":
    main(["--methods", "TRIAD", *sys.argv[1:]])
