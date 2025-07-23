#!/usr/bin/env python3
"""Run all datasets using only the SVD initialisation method.

This is a thin wrapper around ``run_all_methods.py --methods SVD``. Any
additional command line arguments are forwarded to ``run_all_methods``.
"""
from run_all_methods import main
import sys

if __name__ == "__main__":
    main(["--methods", "SVD", *sys.argv[1:]])
