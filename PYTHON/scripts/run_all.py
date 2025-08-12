from __future__ import annotations

import argparse
from pathlib import Path

from . import run_task1, run_task2, run_task3, run_task4


def run_all(gnss: Path, imu: Path, run_name: str) -> None:
    run_task1.run(gnss, imu, run_name)
    run_task2.run(gnss, imu, run_name)
    run_task3.run(gnss, imu, run_name)
    run_task4.run(gnss, imu, run_name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Tasks 1-4")
    parser.add_argument("--gnss", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--run", type=str, required=True)
    args = parser.parse_args()
    run_all(args.gnss, args.imu, args.run)
