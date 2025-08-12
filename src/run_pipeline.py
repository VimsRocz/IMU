"""Unified command line interface for the IMU/GNSS pipeline."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List

import numpy as np

# ensure package import path
_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

from utils.trace_utils import get_logger
from utils.io_loaders import load_imu, load_gnss, load_truth
from utils.timeline import print_timeline
from utils.save_arrays import save_json
from tasks import task1, task2, task3, task4, task5, task6, task7


def parse_args(argv: List[str] | None = None) -> argparse.Namespace:
    """Parse command line arguments."""
    p = argparse.ArgumentParser(description="Run IMU/GNSS fusion pipeline")
    p.add_argument("--imu", required=True, help="Path to IMU .dat file")
    p.add_argument("--gnss", required=True, help="Path to GNSS .csv file")
    p.add_argument("--truth", help="Path to truth STATE file", default=None)
    p.add_argument("--method", choices=["TRIAD", "Davenport", "SVD"], default="TRIAD")
    p.add_argument("--out", default="results", help="Output directory")
    p.add_argument("--save-formats", default="png,pickle,mat")
    p.add_argument("--interactive", choices=["on", "off"], default="off")
    p.add_argument("--debug", choices=["0", "1"], default="0")
    p.add_argument("--resume", choices=["0", "1"], default="1")
    p.add_argument("--force", choices=["0", "1"], default="0")
    p.add_argument("--timestamp", default="auto")
    p.add_argument("--zupt-thresh", type=float, default=None)
    p.add_argument("--timebase", choices=["auto", "posix", "relative"], default="auto")
    return p.parse_args(argv)


def _prepare_run_dir(base: str, case: str, timestamp: str) -> Path:
    run_dir = Path(base) / case / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def infer_case(args: argparse.Namespace) -> str:
    imu_tag = Path(args.imu).stem.upper()
    gnss_tag = Path(args.gnss).stem.upper()
    return f"{imu_tag}_{gnss_tag}_{args.method.upper()}"


def build_runmeta(args, tasks, start, end, imu, gnss, truth):
    return {
        "cmdline": " ".join(sys.argv),
        "args": vars(args),
        "start_time": start,
        "end_time": end,
        "duration_s": end - start,
        "tasks": tasks,
        "inputs": {
            "imu": imu["meta"],
            "gnss": gnss["meta"],
            "truth": truth["meta"] if truth else None,
        },
    }


def main(args: argparse.Namespace | None = None) -> None:
    args = parse_args() if args is None else args
    case = infer_case(args)
    timestamp = (
        time.strftime("%Y%m%d_%H%M%S") if args.timestamp == "auto" else args.timestamp
    )
    run_dir = _prepare_run_dir(args.out, case, timestamp)
    logger = get_logger(run_dir, debug=bool(int(args.debug)))
    logger.info("CMD: %s", " ".join(sys.argv))
    start = time.time()

    imu = load_imu(args.imu, logger)
    gnss = load_gnss(args.gnss, logger)
    truth = load_truth(args.truth, logger, timebase=args.timebase) if args.truth else None

    print_timeline(imu["t"], gnss["t"], truth["t"] if truth else None, logger)

    state = {"case": case, "method": args.method, "run_dir": str(run_dir), "logger": logger, "artifacts": {}}

    t1 = task1.run(state, imu, gnss, args)
    t2 = task2.run(state, imu, args)
    t3 = task3.run(state, t1, t2, args)
    t4 = task4.run(state, imu, gnss, t1, t2, t3, args)
    t5 = task5.run(state, imu, gnss, t1, t2, t3, t4, args)
    if truth:
        t6 = task6.run(state, truth, t4, t5, args)
        t7 = task7.run(state, truth, t4, t5, args)
        tasks_meta = [t1, t2, t3, t4, t5, t6, t7]
    else:
        logger.warning("TRUTH unavailable -> skipping Task 6 & 7")
        tasks_meta = [t1, t2, t3, t4, t5]

    meta = build_runmeta(args, tasks_meta, start, time.time(), imu, gnss, truth)
    save_json(run_dir / "runmeta", meta)


if __name__ == "__main__":  # pragma: no cover
    main()
