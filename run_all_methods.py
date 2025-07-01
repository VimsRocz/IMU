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
from pathlib import Path
import subprocess
import sys
from typing import Iterable, Tuple

HERE = Path(__file__).resolve().parent

try:
    import yaml
except ModuleNotFoundError:  # allow running without PyYAML installed
    yaml = None

DEFAULT_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
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

    results_dir = HERE / "results"
    results_dir.mkdir(exist_ok=True)

    script = HERE / "GNSS_IMU_Fusion.py"

    for (imu, gnss), m in itertools.product(cases, methods):
        imu_path = Path(imu)
        if not imu_path.is_absolute():
            imu_path = HERE / imu_path
        gnss_path = Path(gnss)
        if not gnss_path.is_absolute():
            gnss_path = HERE / gnss_path

        tag = f"{imu_path.stem}_{gnss_path.stem}_{m}"
        log_path = results_dir / f"{tag}.log"
        print(f"\u25B6 {tag}")
        cmd = [
            sys.executable,
            str(script),
            "--imu-file",
            str(imu_path),
            "--gnss-file",
            str(gnss_path),
            "--method",
            m,
        ]
        if args.no_plots:
            cmd.append("--no-plots")
        with open(log_path, "w") as log:
            subprocess.run(
                cmd,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )


if __name__ == "__main__":
    main()
