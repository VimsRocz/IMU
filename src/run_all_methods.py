#!/usr/bin/env python3
"""Run every (IMU, GNSS, method) combo and log the output.

The script processes each IMU/GNSS pair with all selected methods and writes
the console output to ``results/<IMU>_<GNSS>_<method>.log``.  By default the
bundled ``IMU_X`` data sets are used, but a YAML configuration file can
override the data files and the list of methods.

Example ``config.yml``::

    datasets:
      - imu: IMU_X001.dat
        gnss: GNSS_X001.csv
      - imu: IMU_X002.dat
        gnss: GNSS_X002.csv
    methods: [TRIAD, Davenport, SVD]

Run the script with ``--config config.yml`` to process those files.
"""

import argparse
import itertools
import os
import pathlib
import subprocess
import sys
from typing import Iterable, Tuple
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")
os.makedirs('results', exist_ok=True)
logging.info("Ensured 'results/' directory exists.")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

try:
    import yaml
except ModuleNotFoundError:  # allow running without PyYAML installed
    yaml = None

DEFAULT_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X002.dat", "GNSS_X002.csv"),
]

DEFAULT_METHODS = ["TRIAD", "SVD", "Davenport"]


def load_config(path: str):
    """Return (datasets, methods) from a YAML config file."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to use --config")
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    datasets = [
        (item["imu"], item["gnss"]) for item in data.get("datasets", [])
    ] or list(DEFAULT_DATASETS)
    methods = data.get("methods", DEFAULT_METHODS)
    return datasets, methods


def run_case(cmd, log_path):
    """Run a single fusion command and log output live to console and file."""
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        for line in proc.stdout:
            print(line, end="")
            log.write(line)
        proc.wait()
        return proc.returncode


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with multiple datasets and methods",
    )
    parser.add_argument(
        "--config",
        help="YAML file specifying datasets and methods",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation for faster execution",
    )
    args = parser.parse_args(argv)

    if args.config:
        cases, methods = load_config(args.config)
    else:
        cases, methods = list(DEFAULT_DATASETS), list(DEFAULT_METHODS)

    for (imu, gnss), m in itertools.product(cases, methods):
        tag = f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{m}"
        log_path = pathlib.Path("results") / f"{tag}.log"
        print(f"\u25B6 {tag}")
        cmd = [
            sys.executable,
            str(HERE / "GNSS_IMU_Fusion.py"),
            "--imu-file",
            str(ROOT / imu),
            "--gnss-file",
            str(ROOT / gnss),
            "--method",
            m,
        ]
        if args.no_plots:
            cmd.append("--no-plots")
        ret = run_case(cmd, log_path)
        if ret != 0:
            raise subprocess.CalledProcessError(ret, cmd)


if __name__ == "__main__":
    main()
