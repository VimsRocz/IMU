"""Thin wrapper to run the pipeline on dataset X002 using the TRIAD method."""

from __future__ import annotations

import sys

from run_pipeline import main as pipeline_main, parse_args as pipeline_parse


def main(argv: list[str] | None = None) -> None:
    """Execute :mod:`run_pipeline` with X002 defaults."""
    argv = [] if argv is None else list(argv)
    defaults = ["--imu", "IMU_X002.dat", "--gnss", "GNSS_X002.csv", "--method", "TRIAD"]
    args = pipeline_parse(defaults + argv)
    pipeline_main(args)


if __name__ == "__main__":  # pragma: no cover
    main(sys.argv[1:])
