#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method and
validate results when ground truth data is available."""

import subprocess
import sys
from pathlib import Path
import re
import numpy as np
from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames
from utils import ensure_dependencies, ecef_to_geodetic

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

# Install any missing dependencies before running the batch command
ensure_dependencies()

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
results = Path.cwd() / "results"
for mat in results.glob("*_TRIAD_kf_output.mat"):
    m = re.match(r"IMU_(X\d+)_.*_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    truth = HERE / f"STATE_{m.group(1)}.txt"
    if not truth.exists():
        continue
    first = np.loadtxt(truth, comments="#", max_rows=1)
    r0 = first[2:5]
    lat_deg, lon_deg, _ = ecef_to_geodetic(*r0)

    vcmd = [
        sys.executable,
        str(HERE / "validate_with_truth.py"),
        "--est-file",
        str(mat),
        "--truth-file",
        str(truth),
        "--output",
        str(results),
        "--ref-lat",
        str(lat_deg),
        "--ref-lon",
        str(lon_deg),
        "--ref-r0",
        str(r0[0]),
        str(r0[1]),
        str(r0[2]),
    ]
    subprocess.run(vcmd, check=True)
    try:
        est = load_estimate(str(mat))
        m2 = re.match(r"(IMU_\w+)_((?:GNSS_)?\w+)_TRIAD_kf_output", mat.stem)
        if m2:
            imu_file = ROOT / f"{m2.group(1)}.dat"
            gnss_file = ROOT / f"{m2.group(2)}.csv"
            frames = assemble_frames(est, imu_file, gnss_file, truth)
            for frame_name, data in frames.items():
                t_i, p_i, v_i, a_i = data["imu"]
                t_g, p_g, v_g, a_g = data["gnss"]
                t_f, p_f, v_f, a_f = data["fused"]
                truth_data = data.get("truth")
                if truth_data is not None:
                    t_t, p_t, v_t, a_t = truth_data
                else:
                    t_t = p_t = v_t = a_t = None
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
                    t_truth=t_t,
                    pos_truth=p_t,
                    vel_truth=v_t,
                    acc_truth=a_t,
                )
    except Exception as e:
        print(f"Overlay plot failed: {e}")
