#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method and
validate results when ground truth data is available."""

import subprocess
import sys
from pathlib import Path
import re
from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

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
    m = re.match(r"IMU_(.+?)_GNSS_(.+?)_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    ds_id = m.group(1)
    truth = (ROOT / f"IMU_{ds_id}.dat").with_name(f"STATE_{ds_id}.txt")
    if not truth.exists():
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
        imu_file = ROOT / f"IMU_{ds_id}.dat"
        gnss_file = ROOT / f"GNSS_{m.group(2)}.csv"
        frames = assemble_frames(
            est,
            imu_file,
            gnss_file,
            truth_file=str(truth),
        )
        for frame_name, data in frames.items():
            t_i, p_i, v_i, a_i = data["imu"]
            t_g, p_g, v_g, a_g = data["gnss"]
            t_f, p_f, v_f, a_f = data["fused"]
            truth_data = data.get("truth")
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
                truth_data,
            )
    except Exception as e:
        print(f"Overlay plot failed: {e}")
