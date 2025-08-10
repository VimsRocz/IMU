#!/usr/bin/env python3
"""Run all datasets using only the SVD initialisation method.

This is a thin wrapper around ``run_method_only.py --method SVD``. Any
additional command line arguments are forwarded to ``run_method_only``.
"""
from run_method_only import main
import sys

if __name__ == "__main__":
    main(["--method", "SVD", *sys.argv[1:]])
