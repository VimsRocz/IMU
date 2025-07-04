#!/usr/bin/env python3
"""Flexible batch driver for GNSS/IMU fusion experiments."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Dict, Iterable, List

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover - optional dep
    yaml = None

# ---------------------------------------------------------------------------
# Default configuration.  Edit here or provide --config to override.
CONFIG: Dict[str, Any] = {
    "DATASETS": [
        {"imu": "IMU_X001.dat", "gnss": "GNSS_X001.csv"},
        {"imu": "IMU_X002.dat", "gnss": "GNSS_X002.csv"},
    ],
    "METHODS": ["TRIAD", "Davenport", "SVD"],
}

# Path to the core processing script
ROOT = Path(__file__).resolve().parent
MAIN_PROCESSOR = ROOT / "src" / "GNSS_IMU_Fusion.py"
RESULTS_DIR = ROOT / "results"
LOG_DIR = ROOT / "logs"
RESULTS_DIR.mkdir(exist_ok=True)
LOG_DIR.mkdir(exist_ok=True)

try:
    from tqdm import tqdm
except Exception:  # pragma: no cover - optional dep
    def tqdm(x: Iterable, **_: Any) -> Iterable:
        return x

# ---------------------------------------------------------------------------

def load_config(path: Path) -> Dict[str, Any]:
    """Load config from JSON or YAML file."""
    with open(path) as fh:
        if path.suffix in {".yml", ".yaml"}:
            if yaml is None:
                raise RuntimeError("PyYAML required for YAML config files")
            data = yaml.safe_load(fh) or {}
        else:
            data = json.load(fh)
    return {k.upper(): v for k, v in data.items()}


def parse_dataset_args(datasets: List[Dict[str, str]], specs: List[str]) -> List[Dict[str, str]]:
    """Return subset of datasets based on index slice or glob patterns."""
    if not specs:
        return datasets
    selected: List[Dict[str, str]] = []
    from fnmatch import fnmatch

    for spec in specs:
        if ":" in spec:  # slice
            start_s, end_s = spec.split(":", 1)
            start = int(start_s) if start_s else None
            end = int(end_s) if end_s else None
            selected.extend(datasets[slice(start, end)])
        elif spec.isdigit():
            idx = int(spec)
            if 0 <= idx < len(datasets):
                selected.append(datasets[idx])
        else:
            for ds in datasets:
                if fnmatch(Path(ds["imu"]).name, spec) or fnmatch(
                    Path(ds["gnss"]).name, spec
                ):
                    if ds not in selected:
                        selected.append(ds)
    return selected


def run_case(dataset: Dict[str, str], method: str) -> Dict[str, Any]:
    """Run MAIN_PROCESSOR for one dataset/method combo."""
    imu = dataset["imu"]
    gnss = dataset["gnss"]
    tag = f"{Path(imu).stem}_{Path(gnss).stem}"
    res_dir = RESULTS_DIR / tag / method
    res_dir.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{tag}_{method}.log"
    cmd = [
        sys.executable,
        str(MAIN_PROCESSOR),
        "--imu-file",
        str(ROOT / imu),
        "--gnss-file",
        str(ROOT / gnss),
        "--method",
        method,
    ]
    start = time.time()
    with open(log_path, "w") as log:
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
    runtime = time.time() - start
    return {
        "tag": tag,
        "method": method,
        "return_code": proc.returncode,
        "runtime_sec": runtime,
        "log_path": str(log_path),
    }


def main(argv: Iterable[str] | None = None) -> None:
    """Entry point for command line execution."""
    parser = argparse.ArgumentParser(description="Batch-run GNSS/IMU experiments")
    parser.add_argument("--config", type=Path, help="YAML/JSON config file")
    parser.add_argument("--methods", nargs="*", help="Subset of methods to run")
    parser.add_argument(
        "--datasets",
        nargs="*",
        metavar="SPEC",
        help="Dataset indices or glob patterns",
    )
    parser.add_argument("--jobs", type=int, default=1, help="Parallel workers")
    args = parser.parse_args(argv)

    config = dict(CONFIG)
    if args.config:
        config.update(load_config(args.config))

    datasets = config.get("DATASETS", [])
    methods = config.get("METHODS", [])

    if args.methods:
        methods = [m for m in methods if m in args.methods]

    datasets = parse_dataset_args(datasets, args.datasets or [])

    cases = [
        (ds, m)
        for ds in datasets
        for m in methods
    ]

    results: List[Dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=args.jobs) as exe:
        futures = {exe.submit(run_case, ds, m): (ds, m) for ds, m in cases}
        for fut in tqdm(as_completed(futures), total=len(futures), desc="Runs"):
            res = fut.result()
            results.append(res)

    # CSV summary --------------------------------------------------------------
    import csv

    summary_path = RESULTS_DIR / "run_summary.csv"
    with open(summary_path, "w", newline="") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=["tag", "method", "return_code", "runtime_sec", "log_path"],
        )
        writer.writeheader()
        writer.writerows(results)

    # Pretty-print -------------------------------------------------------------
    try:
        from tabulate import tabulate
    except Exception:  # pragma: no cover - optional dep
        for r in results:
            print(r)
    else:
        rows = [
            [r["tag"], r["method"], r["return_code"], f"{r['runtime_sec']:.2f}", r["log_path"]]
            for r in results
        ]
        print(tabulate(rows, headers=["tag", "method", "ret", "sec", "log"]))


if __name__ == "__main__":  # pragma: no cover
    main()
