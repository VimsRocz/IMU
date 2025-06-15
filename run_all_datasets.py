#!/usr/bin/env python3
"""
Batch-runner for all IMU/GNSS sets and all attitude-initialisation methods.
Writes one log file per run in ./logs and prints a short SUMMARY line
that `summarise_runs.py` can aggregate later.
"""

import pathlib
import subprocess
import datetime
import itertools
import sys
import argparse
import re
import time

from rich.console import Console
from rich.table import Table
from tqdm import tqdm

HERE     = pathlib.Path(__file__).resolve().parent
SCRIPT   = HERE / "GNSS_IMU_Fusion.py"
LOG_DIR  = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)

DATASETS = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
    ("IMU_X002.dat", "GNSS_X002.csv"),
    ("IMU_X003.dat", "GNSS_X002.csv"),   # <- note the GNSS swap
]

METHODS  = ["TRIAD", "Davenport", "SVD"]

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")

def run_one(imu, gnss, method, verbose=False):
    ts    = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log   = LOG_DIR / f"{imu}_{gnss}_{method}_{ts}.log"
    cmd   = [
        sys.executable,
        SCRIPT,
        "--imu-file",
        imu,
        "--gnss-file",
        gnss,
        "--method",
        method,
    ]
    if verbose:
        cmd.append("--verbose")
    summary_line = None
    with log.open("w") as fh:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:  # live-stream to console & file
            print(line, end="")
            fh.write(line)
            m = SUMMARY_RE.search(line)
            if m:
                summary_line = m.group(1)
    if proc.wait() != 0:
        raise RuntimeError(f"{cmd} failed, see {log}")
    return summary_line

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action="store_true", help="Print detailed debug info")
    args = parser.parse_args()

    here_files = {p.name for p in HERE.iterdir()}
    cases = [(imu, gnss, m) for (imu, gnss) in DATASETS for m in METHODS]
    console = Console()
    table = Table(title="Fusion Results")
    table.add_column("Method")
    table.add_column("RMSE (m)", justify="right")
    table.add_column("Final Error (m)", justify="right")
    table.add_column("Runtime (s)", justify="right")

    for imu, gnss, method in tqdm(cases, desc="All cases"):
        if imu not in here_files or gnss not in here_files:
            raise FileNotFoundError(f"Missing {imu} or {gnss} in {HERE}")
        start = time.time()
        summary = run_one(imu, gnss, method, verbose=args.verbose)
        runtime = time.time() - start
        if summary:
            # split on key=value pairs allowing spaces after '=' for
            # formatted numbers like "final_pos=  10.84m"
            kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
            table.add_row(
                kv.get("method", method),
                kv.get("rmse_pos", "n/a").replace("m", ""),
                kv.get("final_pos", "n/a").replace("m", ""),
                f"{runtime:.1f}"
            )

    console.print(table)

if __name__ == "__main__":
    main()
