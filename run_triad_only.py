#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method.

Optionally validate the Kalman filter output against a reference state
with ``--truth-file PATH``. The file is passed to ``validate_with_truth``
and skipped if not found.
"""

import argparse
import subprocess
import sys
import pathlib
import re
import numpy as np
from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames
from utils import ecef_to_geodetic
import pandas as pd

HERE = pathlib.Path(__file__).resolve().parent

ap = argparse.ArgumentParser(description="Run TRIAD initialisation on all datasets")
ap.add_argument("--truth-file", help="Reference state file for validation")
args, rest = ap.parse_known_args()

# --- Run the batch processor -------------------------------------------------
cmd = [
    sys.executable,
    str(HERE / "run_all_datasets.py"),
    "--method",
    "TRIAD",
    *rest,
]
subprocess.run(cmd, check=True)

# --- Validate results when STATE_<id>.txt exists -----------------------------
results = HERE / "results"
truth_arg = pathlib.Path(args.truth_file).expanduser() if args.truth_file else None
if truth_arg and not truth_arg.exists():
    print(f"Warning: truth file {truth_arg} not found, skipping reference overlay")
    truth_arg = None

for mat in results.glob("*_TRIAD_kf_output.mat"):
    m = re.match(r"IMU_(X\d+)(?:_small)?_.*_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    dataset = m.group(1)
    if truth_arg is not None:
        truth = truth_arg
    else:
        candidates = [
            HERE / f"STATE_{dataset}_small.txt",
            HERE / f"STATE_{dataset}.txt",
        ]
        if not any(c.exists() for c in candidates):
            candidates.extend(
                [HERE / "STATE_X001_small.txt", HERE / "STATE_X001.txt"]
            )
        truth = next((c for c in candidates if c.exists()), None)
    if truth is None:
        if args.truth_file:
            print(f"Warning: reference file {truth_arg} not found, skipping validation")
        else:
            print(f"Warning: no truth file for {dataset}, skipping validation")
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
    try:
        est = load_estimate(str(mat))
        m2 = re.match(r"(IMU_\w+)_GNSS_(\w+)_TRIAD_kf_output", mat.stem)
        if m2:
            imu_file = HERE / f"{m2.group(1)}.dat"
            gnss_file = HERE / f"{m2.group(2)}.csv"
            gnss = pd.read_csv(gnss_file, nrows=1)
            x0, y0, z0 = (
                gnss[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].iloc[0].to_numpy()
            )
            lat0, lon0, _ = ecef_to_geodetic(x0, y0, z0)
            frames = assemble_frames(
                est,
                imu_file,
                gnss_file,
                np.deg2rad(lat0),
                np.deg2rad(lon0),
                np.array([x0, y0, z0]),
            )
            for frame_name, data in frames.items():
                t_i, p_i, v_i, a_i = data["imu"]
                t_g, p_g, v_g, a_g = data["gnss"]
                t_f, p_f, v_f, a_f = data["fused"]
                plot_overlay(
                    frame_name,
                    "TRIAD",
                    t_i,
                    p_i,
                    v_i,
                    a_i,
                    t_g,
                    p_g,
                    v_g,
                    a_g,
                    t_f,
                    p_f,
                    v_f,
                    a_f,
                    results,
                )
    except Exception as e:
        print(f"Overlay plot failed: {e}")
