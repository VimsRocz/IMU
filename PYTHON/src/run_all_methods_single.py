#!/usr/bin/env python3
"""Run all attitude initialisation methods on a single dataset.

This helper executes ``run_triad_only.py``, ``run_davenport_only.py`` and
``run_svd_only.py`` sequentially for the provided IMU and GNSS files. All
subprocess output is forwarded to the console in real time. Lines beginning
with ``[SUMMARY]`` are collected and printed again at the end so results from
all methods can be compared easily.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

SCRIPT_MAP = {
    "TRIAD": "run_triad_only.py",
    "Davenport": "run_davenport_only.py",
    "SVD": "run_svd_only.py",
}

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def run_method(script: str, imu: str, gnss: str, dataset: str, truth: str | None):
    cmd = [
        sys.executable,
        str(Path(__file__).resolve().parent / script),
        "--imu",
        imu,
        "--gnss",
        gnss,
        "--dataset",
        dataset,
    ]
    if truth:
        cmd += ["--truth", truth, "--allow-truth-mismatch"]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    summaries: list[str] = []
    assert proc.stdout is not None
    for line in proc.stdout:
        print(line, end="")
        m = SUMMARY_RE.search(line)
        if m:
            summaries.append(m.group(1))
    proc.wait()
    return proc.returncode, summaries


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run TRIAD, Davenport and SVD on one dataset",
    )
    parser.add_argument("--imu", required=True, help="Path to IMU .dat file")
    parser.add_argument("--gnss", required=True, help="Path to GNSS .csv file")
    parser.add_argument("--dataset", default="X002", help="Dataset identifier")
    parser.add_argument("--truth", help="Optional truth file for evaluation")
    args = parser.parse_args(argv)

    all_summaries: list[tuple[str, str]] = []
    for method, script in SCRIPT_MAP.items():
        print(f"\n=== {method} ===")
        ret, summaries = run_method(script, args.imu, args.gnss, args.dataset, args.truth)
        if ret != 0:
            print(f"\n{method} run failed with exit code {ret}")
        for s in summaries:
            all_summaries.append((method, s))

    if all_summaries:
        print("\n=== Summary ===")
        for method, line in all_summaries:
            print(f"{method}: {line}")

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

