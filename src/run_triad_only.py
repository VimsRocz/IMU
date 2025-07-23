#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method.

This script forwards any additional command line arguments to
``run_method_only.py`` with ``--method TRIAD`` so that the behaviour
matches ``run_all_methods.py`` when the TRIAD method is selected. If
``STATE_X*.txt`` truth logs are available the batch runner automatically
performs the Task 6 overlay and Task 7 evaluation, matching the MATLAB
workflow.

Usage
-----
    python src/run_triad_only.py [options]
"""

from run_method_only import main
import sys

if __name__ == "__main__":
    main(["--method", "TRIAD", *sys.argv[1:]])
