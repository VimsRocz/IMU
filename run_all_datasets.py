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

HERE = pathlib.Path(__file__).resolve().parent


def ensure_dependencies():
    """Install required packages if they're missing."""
    try:  # check a couple of external deps
        import tabulate  # noqa: F401
        import tqdm  # noqa: F401
    except ModuleNotFoundError:
        print("Installing Python dependencies ...")
        req = HERE / "requirements.txt"
        subprocess.check_call([
            sys.executable,
            "-m",
            "pip",
            "install",
            "-r",
            str(req),
        ])


ensure_dependencies()

from tabulate import tabulate
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
    summary_lines = []
    with log.open("w") as fh:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:  # live-stream to console & file
            print(line, end="")
            fh.write(line)
            m = SUMMARY_RE.search(line)
            if m:
                summary_lines.append(m.group(1))
    if proc.wait() != 0:
        raise RuntimeError(f"{cmd} failed, see {log}")
    return summary_lines

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action="store_true", help="Print detailed debug info")
    parser.add_argument("--datasets", default="ALL",
                        help="Comma separated dataset IDs (e.g. X001,X002) or ALL")
    parser.add_argument('--method', choices=['TRIAD','Davenport','SVD'],
                        default='Davenport')
    args = parser.parse_args()

    here_files = {p.name for p in HERE.iterdir()}

    if args.datasets.upper() == "ALL":
        datasets = DATASETS
    else:
        ids = {d.strip() for d in args.datasets.split(',')}
        datasets = [p for p in DATASETS if pathlib.Path(p[0]).stem.split('_')[1] in ids]

    method = args.method
    cases = [(imu, gnss, method) for (imu, gnss) in datasets]
    fusion_results = []

    for imu, gnss, method in tqdm(cases, desc="All cases"):
        if imu not in here_files or gnss not in here_files:
            raise FileNotFoundError(f"Missing {imu} or {gnss} in {HERE}")
        start = time.time()
        summaries = run_one(imu, gnss, method, verbose=args.verbose)
        runtime = time.time() - start
        for summary in summaries:
            kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
            fusion_results.append({
                "dataset"  : pathlib.Path(imu).stem.split("_")[1],
                "method"   : kv.get("method", method),
                "rmse_pos" : float(kv.get("rmse_pos", "nan").replace("m", "")),
                "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                "runtime"  : runtime,
            })

    # --- nicely formatted summary table --------------------------------------
    key_order = {m: i for i, m in enumerate(["TRIAD", "Davenport", "SVD"])}
    fusion_results.sort(key=lambda r: (r["dataset"], key_order[r["method"]]))
    rows = [
        [e["dataset"], e["method"], e["rmse_pos"], e["final_pos"], e["runtime"]]
        for e in fusion_results
    ]
    print(tabulate(
        rows,
        headers=["Dataset", "Method", "RMSEpos [m]", "End-Error [m]", "Runtime [s]"],
        floatfmt=".2f",
    ))

    # Optional CSV export for easier analysis
    import pandas as pd
    df = pd.DataFrame(rows, columns=["Dataset", "Method", "RMSEpos_m", "EndErr_m", "Runtime_s"])
    df.to_csv(HERE / "results" / "summary.csv", index=False)


if __name__ == "__main__":
    main()
