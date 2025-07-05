#!/usr/bin/env python3
"""Validate state estimator results against ground truth.

This helper script checks for the presence of a ground truth file and, when
available, runs ``validate_with_truth.py`` to generate the standard error
metrics and overlay plots.

Example usage::

    python3 validate_and_plot.py --est-file results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat \
        --truth-file STATE_X001.txt --output results
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--est-file", required=True, help="state estimator output (.mat or .npz)"
    )
    parser.add_argument(
        "--truth-file", required=True, help="ground truth trajectory file"
    )
    parser.add_argument(
        "--output", default="results", help="directory for validation results"
    )
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)
    print(f"Ensured '{output_path}/' directory exists.")

    est_path = Path(args.est_file)
    truth_path = Path(args.truth_file)

    if not est_path.exists():
        print(f"Estimator result '{est_path}' not found.")
        return

    if not truth_path.exists():
        print(f"Truth file '{truth_path}' not found, skipping validation.")
        return

    here = Path(__file__).resolve().parent
    validate_script = here / "src" / "validate_with_truth.py"

    cmd = [
        sys.executable,
        str(validate_script),
        "--est-file",
        str(est_path),
        "--truth-file",
        str(truth_path),
        "--output",
        str(output_path),
    ]

    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
