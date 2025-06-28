#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method.

If results for dataset X001 are present afterwards, they will be validated with
``validate_with_truth.py`` automatically."""
import subprocess
import sys
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
cmd = [sys.executable, str(HERE / "run_all_datasets.py"), "--method", "TRIAD"] + sys.argv[1:]
subprocess.run(cmd, check=True)

# Validate TRIAD results for dataset X001 if the output files exist
est_file = HERE / "results" / "IMU_X001_GNSS_X001_TRIAD_kf_output.mat"
truth_file = HERE / "STATE_X001.txt"
if est_file.exists() and truth_file.exists():
    subprocess.run(
        [
            sys.executable,
            str(HERE / "validate_with_truth.py"),
            "--est-file",
            str(est_file),
            "--truth-file",
            str(truth_file),
        ],
        check=True,
    )
else:
    print(
        f"Warning: skipping validation, missing {est_file} or {truth_file}",
        file=sys.stderr,
    )
