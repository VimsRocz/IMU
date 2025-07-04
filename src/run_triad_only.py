#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method and
validate results when ground truth data is available."""

import subprocess
import sys
from pathlib import Path
import re
import numpy as np
import pandas as pd
from tabulate import tabulate
from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames
from utils import ensure_dependencies
from pyproj import Transformer

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

# Collect validation results for a final summary table
summary = []

# --- Validate results when STATE_<id>.txt exists -----------------------------
results = Path.cwd() / "results"
for mat in results.glob("*_TRIAD_kf_output.mat"):
    m = re.match(r"IMU_(X\d+)_.*_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    truth = HERE / f"STATE_{m.group(1)}.txt"
    if not truth.exists():
        print(f"Warning: Truth file for {m.group(1)} not found")
        continue
    first = np.loadtxt(truth, comments="#", max_rows=1)
    r0 = first[2:5]
    expected_lat = expected_lon = None
    if m.group(1) == "X001":
        r0 = np.array([-3729050.8173, 3935675.6126, -3348394.2576])
        expected_lat, expected_lon = -32.026554, 133.455801
    transformer = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
    lon_deg, lat_deg, _ = transformer.transform(*r0)
    msg = f"Debug: Computed lat={lat_deg:.6f}°, lon={lon_deg:.6f}° from r0={r0}"
    if expected_lat is not None:
        msg += f" (expected lat={expected_lat:.6f}°, lon={expected_lon:.6f}°)"
    print(msg)

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
    proc = subprocess.run(vcmd, check=True, capture_output=True, text=True)
    print(proc.stdout)
    # parse validation summary metrics
    metrics = {}
    for line in proc.stdout.splitlines():
        m_val = re.search(r"Final position error:\s*([0-9.eE+-]+)", line)
        if m_val:
            metrics["final_pos"] = float(m_val.group(1))
        m_val = re.search(r"RMSE position error:\s*([0-9.eE+-]+)", line)
        if m_val:
            metrics["rmse_pos"] = float(m_val.group(1))
        m_val = re.search(r"Final velocity error:\s*([0-9.eE+-]+)", line)
        if m_val:
            metrics["final_vel"] = float(m_val.group(1))
        m_val = re.search(r"RMSE velocity error:\s*([0-9.eE+-]+)", line)
        if m_val:
            metrics["rmse_vel"] = float(m_val.group(1))
        m_val = re.search(r"Final attitude error:\s*([0-9.eE+-]+)", line)
        if m_val:
            metrics["final_att"] = float(m_val.group(1))
        m_val = re.search(r"RMSE attitude error:\s*([0-9.eE+-]+)", line)
        if m_val:
            metrics["rmse_att"] = float(m_val.group(1))
    try:
        truth_data = np.loadtxt(truth)
        t_truth = truth_data[:, 1]
        est = load_estimate(str(mat), times=t_truth)
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

            npz_path = mat.with_suffix('.npz')
            q0 = [float('nan')]*4
            P0 = [float('nan')]*3
            try:
                npz = np.load(npz_path, allow_pickle=True)
                if 'attitude_q' in npz:
                    q0 = npz['attitude_q'][0]
                if 'P_hist' in npz:
                    P0 = np.diagonal(npz['P_hist'][0])[:3]
            except Exception:
                pass

            summary.append({
                'dataset': m.group(1),
                **metrics,
                'q0_w': q0[0],
                'q0_x': q0[1],
                'q0_y': q0[2],
                'q0_z': q0[3],
                'Pxx': P0[0],
                'Pyy': P0[1],
                'Pzz': P0[2],
            })
    except Exception as e:
        print(f"Overlay plot failed: {e}")

if summary:
    rows = [
        [s.get('dataset'), s.get('rmse_pos'), s.get('final_pos'), s.get('rmse_vel'),
         s.get('final_vel'), s.get('rmse_att'), s.get('final_att'),
         s.get('q0_w'), s.get('q0_x'), s.get('q0_y'), s.get('q0_z'),
         s.get('Pxx'), s.get('Pyy'), s.get('Pzz')]
        for s in summary
    ]
    headers = [
        'Dataset', 'RMSEpos[m]', 'FinalPos[m]', 'RMSEvel[m/s]', 'FinalVel[m/s]',
        'RMSEatt[deg]', 'FinalAtt[deg]', 'q0_w', 'q0_x', 'q0_y', 'q0_z',
        'Pxx', 'Pyy', 'Pzz'
    ]
    print(tabulate(rows, headers=headers, floatfmt='.3f'))
    df = pd.DataFrame(rows, columns=headers)
    df.to_csv(results / 'summary_truth.csv', index=False)

