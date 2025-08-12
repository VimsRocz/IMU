"""Stub mirroring ``MATLAB/src/utils/print_timeline_matlab.m``."""

from __future__ import annotations

from .timeline import print_timeline_summary


def print_timeline_matlab(run_id_str: str, imu_path: str, gnss_path: str, truth_path: str, out_dir: str):
    """Delegate to :func:`print_timeline_summary` for cross-language parity."""

    return print_timeline_summary(run_id_str, imu_path, gnss_path, truth_path, out_dir)


__all__ = ["print_timeline_matlab"]

