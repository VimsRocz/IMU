#!/usr/bin/env python3
"""
Analyze attitude divergence time vs. dataset length using existing outputs.

This tool loads a filter estimate (MAT/NPZ) and truth (STATE_X*.txt),
computes the quaternion angle error, detects the time when the attitude
diverges (exceeds a robust threshold for a sustained period), and repeats
the measurement across a set of truncated end-times. It does not rerun the
filter; it evaluates divergence on the provided estimate up to each length.

For a rigorous test of dependency on dataset length, rerun the filter for
each length and pass the corresponding outputs to this tool.
"""

import argparse
import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from tabulate import tabulate

from validate_with_truth import load_estimate


def detect_divergence(tvec, err_series_deg, base_window_s=60.0, threshold_deg=5.0, hold_s=5.0):
    if len(tvec) == 0:
        return None, None
    t0 = float(tvec[0])
    base_mask = (tvec - t0) <= base_window_s
    if not np.any(base_mask):
        base_mask = np.ones_like(tvec, dtype=bool)
    base = err_series_deg[base_mask]
    base_med = float(np.median(base))
    base_std = float(np.std(base))
    thr = max(threshold_deg, base_med + 3.0 * base_std)
    dt = np.diff(tvec, prepend=tvec[0])
    above = err_series_deg >= thr
    run_time = 0.0
    for i in range(len(tvec)):
        if above[i]:
            run_time += dt[i]
            if run_time >= hold_s:
                j = i
                while j > 0 and above[j - 1]:
                    j -= 1
                return float(tvec[j]), float(thr)
        else:
            run_time = 0.0
    return None, float(thr)


def compute_angle_error_deg(q_est_wxyz, q_true_wxyz):
    r_true = R.from_quat(q_true_wxyz[:, [1, 2, 3, 0]])
    r_est = R.from_quat(q_est_wxyz[:, [1, 2, 3, 0]])
    r_err = r_est * r_true.inv()
    err_quat = r_err.as_quat()[:, [3, 0, 1, 2]]
    err_angles = 2 * np.arccos(np.clip(np.abs(err_quat[:, 0]), -1.0, 1.0))
    return np.degrees(err_angles)


def main():
    ap = argparse.ArgumentParser(description="Divergence vs dataset length")
    ap.add_argument("--est-file", required=True, help="Filter output (.mat or .npz)")
    ap.add_argument("--truth-file", required=True, help="Truth STATE_X file")
    ap.add_argument(
        "--lengths",
        default="120,300,600,900,1200",
        help="Comma-separated lengths [s] to evaluate",
    )
    ap.add_argument("--output", default="results", help="Output directory")
    args = ap.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    truth = np.loadtxt(args.truth_file)
    t_truth = truth[:, 1]
    q_true = truth[:, 8:12]

    lengths = []
    for tok in str(args.lengths).split(','):
        tok = tok.strip()
        if not tok:
            continue
        try:
            lengths.append(float(tok))
        except ValueError:
            pass
    lengths = sorted(set(lengths))
    if not lengths:
        print("No valid lengths provided.")
        return

    results = []
    for L in lengths:
        t_end = t_truth[0] + L
        mask = t_truth <= t_end
        t_eval = t_truth[mask]
        if len(t_eval) < 2:
            results.append([L, None, None])
            continue
        est = load_estimate(args.est_file, times=t_eval)
        if est.get("quat") is None:
            results.append([L, None, None])
            continue
        q_est = np.asarray(est["quat"])[: len(t_eval)]
        q_t = q_true[: len(t_eval)]
        err_deg = compute_angle_error_deg(q_est, q_t)
        t_div, thr = detect_divergence(t_eval, err_deg)
        results.append([L, t_div, thr])

    headers = ["Length_s", "DivergenceTime_s", "Threshold_deg"]
    print(tabulate(results, headers=headers, floatfmt=".3f"))
    out_path = out_dir / "divergence_vs_length.csv"
    with out_path.open("w") as f:
        f.write(",".join(headers) + "\n")
        for row in results:
            f.write(",".join("" if v is None else f"{v:.6f}" for v in row) + "\n")
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()

