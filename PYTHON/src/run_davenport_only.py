#!/usr/bin/env python3
"""Run all datasets using only the Davenport initialisation method.

This script simply calls ``run_method_only.py --method Davenport`` and
passes through any extra command line arguments.
"""
from run_method_only import main
import sys

if __name__ == "__main__":
    main(["--method", "Davenport", *sys.argv[1:]])
