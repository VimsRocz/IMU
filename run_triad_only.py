#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method."""
import subprocess
import sys
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
cmd = [sys.executable, str(HERE / "run_all_datasets.py"), "--method", "TRIAD"] + sys.argv[1:]
subprocess.run(cmd, check=True)
