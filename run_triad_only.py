#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method and
validate results when ground truth data is available."""

import subprocess
import sys
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent

# --- Run the batch processor -------------------------------------------------
cmd = [
    sys.executable,
    str(HERE / "run_all_datasets.py"),
    "--method",
    "TRIAD",
    *sys.argv[1:],
]
subprocess.run(cmd, check=True)

# --- Validate results when STATE_<id>.txt exists -----------------------------
results = HERE / "results"
for mat in results.glob("*_TRIAD_kf_output.mat"):
    m = re.match(r"IMU_(X\d+)(?:_small)?_.*_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    dataset = m.group(1)
    candidates = [
        HERE / f"STATE_{dataset}_small.txt",
        HERE / f"STATE_{dataset}.txt",
    ]
    truth = next((c for c in candidates if c.exists()), None)
    if truth is None:
        continue
    vcmd = [
        sys.executable,
        str(HERE / "validate_with_truth.py"),
        "--est-file",
        str(mat),
        "--truth-file",
        str(truth),
        "--output",
        str(results),
    ]
    subprocess.run(vcmd, check=True)

