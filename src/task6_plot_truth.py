#!/usr/bin/env python3
"""Task 6 – Overlay fused results with the true trajectory.

This helper script loads the Kalman filter output from Task 5 together with
the corresponding ground truth file and saves overlay figures in the same
style as Task 5 but with the truth trajectory included.
"""

import argparse
import re
from pathlib import Path

from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Plot fused IMU/GNSS output with the reference trajectory."
        )
    )
    parser.add_argument(
        "--est-file",
        required=True,
        help="Path to <IMU>_<GNSS>_<METHOD>_kf_output.mat or .npz",
    )
    parser.add_argument(
        "--truth-file",
        required=True,
        help="STATE_X001.txt or similar ground truth file",
    )
    parser.add_argument(
        "--output",
        default="results",
        help="Directory for the generated PDFs",
    )
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    est_path = Path(args.est_file)
    m = re.match(r"(IMU_\w+)_(GNSS_\w+)_([A-Za-z]+)_kf_output", est_path.stem)
    if not m:
        raise ValueError("Estimator filename must follow <IMU>_<GNSS>_<METHOD>_kf_output.*")
    imu_file = Path(f"{m.group(1)}.dat")
    gnss_file = Path(f"{m.group(2)}.csv")
    method = m.group(3)

    est = load_estimate(str(est_path))
    frames = assemble_frames(est, imu_file, gnss_file, args.truth_file)

    for frame_name, data in frames.items():
        t_i, p_i, v_i, a_i = data["imu"]
        t_g, p_g, v_g, a_g = data["gnss"]
        t_f, p_f, v_f, a_f = data["fused"]
        truth = data.get("truth")
        plot_overlay(
            frame_name,
            method,
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
            out_dir,
            truth,
        )


if __name__ == "__main__":
    main()
