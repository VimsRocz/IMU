#!/usr/bin/env python3
"""Run all datasets using only the Davenport initialisation method.

This script simply calls ``run_all_methods.py --methods Davenport`` and
passes through any extra command line arguments.
"""
from run_all_methods import main
import sys

if __name__ == "__main__":
    main(["--methods", "Davenport", *sys.argv[1:]])
