#!/usr/bin/env python3
"""Validate all estimator outputs in a directory.

This helper looks for files ending in ``_kf_output.mat`` or ``_kf_output.npz``
inside the results folder and runs :mod:`validate_and_plot` for each one using
the matching ``STATE_X*.txt`` truth file when available.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=Path("results"),
        help="directory containing estimator outputs",
    )
    parser.add_argument(
        "--truth-dir",
        type=Path,
        default=Path("."),
        help="directory containing STATE_X*.txt files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results"),
        help="directory for validation results",
    )
    args = parser.parse_args(argv)

    if not args.results_dir.exists():
        print(f"Results directory '{args.results_dir}' not found.")
        return

    for est_file in sorted(args.results_dir.glob("*_kf_output.*")):
        if est_file.suffix not in {".mat", ".npz"}:
            continue
        m = re.search(r"IMU_(X\d+)", est_file.name)
        if not m:
            continue
        dataset_id = m.group(1)
        truth_file = args.truth_dir / f"STATE_{dataset_id}.txt"
        if not truth_file.exists():
            print(
                f"Truth file '{truth_file}' not found, skipping {est_file.name}."
            )
            continue
        cmd = [
            sys.executable,
            str(Path(__file__).resolve().parent / "validate_and_plot.py"),
            "--est-file",
            str(est_file),
            "--truth-file",
            str(truth_file),
            "--output",
            str(args.output),
        ]
        print("Running", " ".join(cmd))
        subprocess.run(cmd, check=True)


if __name__ == "__main__":  # pragma: no cover
    main()
