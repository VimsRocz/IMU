#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method and
validate results when ground truth data is available."""

import subprocess
import sys
import pathlib
import re
from plot_overlay import plot_overlay
from plots import plot_frame
from validate_with_truth import (
    load_estimate,
    assemble_frames,
    prepare_truth_frames,
)
from utils import ecef_to_geodetic
import pandas as pd
import argparse
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent

ap = argparse.ArgumentParser(description="Run TRIAD and optional validation")
ap.add_argument(
    "--truth-file",
    help="CSV or TXT file with reference state to overlay",
)
args, other = ap.parse_known_args()

# --- Run the batch processor -------------------------------------------------
cmd = [
    sys.executable,
    str(HERE / "run_all_datasets.py"),
    "--method",
    "TRIAD",
    *other,
]
subprocess.run(cmd, check=True)

# --- Validate results when STATE_<id>.txt exists -----------------------------
results = HERE / "results"
for mat in results.glob("*_TRIAD_kf_output.mat"):
    m = re.match(r"IMU_(X\d+)(?:_small)?_.*_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    dataset = m.group(1)
    truth = None
    if args.truth_file:
        p = pathlib.Path(args.truth_file)
        if p.exists():
            truth = p
        else:
            print(f"Warning: truth file {p} not found; skipping reference overlay")
    if truth is None:
        candidates = [
            HERE / f"STATE_{dataset}_small.txt",
            HERE / f"STATE_{dataset}.txt",
        ]
        truth = next((c for c in candidates if c.exists()), None)
    if truth is None:
        print(f"Warning: no truth file for {dataset}; skipping reference overlay")
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
                truth_frames = None
                if truth is not None:
                    try:
                        truth_data = np.loadtxt(truth)
                        truth_frames = prepare_truth_frames(
                            truth_data,
                            np.deg2rad(lat0),
                            np.deg2rad(lon0),
                            np.array([x0, y0, z0]),
                            t_f,
                        )
                    except Exception as e:
                        print(f"Truth preparation failed: {e}")
                try:
                    plot_frame(
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
                        None if truth_frames is None else truth_frames.get(frame_name),
                    )
                except Exception as e:
                    print(f"Frame plot failed: {e}")
    except Exception as e:
        print(f"Overlay plot failed: {e}")
