"""Timeline summary helpers.

This module prints a concise timeline summary for IMU, GNSS and optional
truth files. A matching MATLAB implementation lives in
``MATLAB/src/utils/timeline_summary.m``.

Functions
---------
print_timeline_summary
    Read dataset files, emit a console summary and write ``*_timeline``
    ``.txt`` and ``.json`` files.
"""

from __future__ import annotations

import json
import os
from typing import List

import numpy as np
import pandas as pd


def _unwrap_subsec(v: np.ndarray) -> np.ndarray:
    """Unwrap a [0,1) sub-second counter that resets each second."""

    v = np.asarray(v).astype(float).ravel()
    out = np.empty_like(v)
    wraps = 0.0
    out[0] = v[0]
    for i in range(1, v.size):
        dv = v[i] - v[i - 1]
        if dv < -0.5:  # rolled over
            wraps += 1.0
        out[i] = v[i] + wraps
    return out


def _detect_imu_time(imu_path: str, dt_fallback: float = 0.0025,
                     notes: List[str] | None = None) -> np.ndarray:
    """Return inferred IMU time vector from ``imu_path``."""

    if notes is None:
        notes = []
    try:
        # be liberal: whitespace or comma-delimited, no header
        try:
            M = pd.read_csv(imu_path, header=None).values
        except Exception:
            M = np.loadtxt(imu_path)
    except Exception as e:
        notes.append(f"IMU: failed to read ({e}); fallback uniform dt={dt_fallback}")
        n = 500000
        return np.arange(n) * dt_fallback

    # 1) look for a [0,1) sub-second column near the end
    for c in range(M.shape[1] - 1, max(-1, M.shape[1] - 4), -1):
        col = M[:, c]
        if np.isfinite(col).all() and (col.min() >= 0) and (col.max() < 1):
            notes.append(f"IMU: used sub-second column {c} with unwrap()")
            return _unwrap_subsec(col)

    # 2) look for a monotonic-ish small step column near the front
    for c in range(min(6, M.shape[1])):
        col = M[:, c]
        if np.isfinite(col).all():
            d = np.diff(col)
            med = np.median(np.abs(d))
            if 1e-4 < med < 1.0:
                notes.append(f"IMU: used time-like column {c} (median dt={med:.6f})")
                return col.astype(float).ravel()

    # 3) fallback to constant rate
    notes.append(f"IMU: no time column; fallback uniform dt={dt_fallback}")
    return np.arange(M.shape[0]) * dt_fallback


def _timeline_stats(t: np.ndarray) -> dict:
    """Return timing statistics for a time vector ``t``."""

    t = np.asarray(t).astype(float).ravel()
    n = len(t)
    if n <= 1:
        return dict(
            n=n,
            hz=float("nan"),
            dt_med=float("nan"),
            dt_min=float("nan"),
            dt_max=float("nan"),
            dur=0.0,
            t0=(t[0] if n else float("nan")),
            t1=(t[-1] if n else float("nan")),
            monotonic=False,
        )
    dt = np.diff(t)
    dt_med = float(np.median(dt))
    hz = (1.0 / dt_med) if dt_med > 0 else float("inf")
    return dict(
        n=int(n),
        hz=hz,
        dt_med=dt_med,
        dt_min=float(np.min(dt)),
        dt_max=float(np.max(dt)),
        dur=float(t[-1] - t[0]),
        t0=float(t[0]),
        t1=float(t[-1]),
        monotonic=bool(np.all(dt > 0)),
    )


def _gnss_time(gnss_path: str, notes: List[str] | None = None) -> np.ndarray:
    """Return GNSS time vector from CSV file at ``gnss_path``."""

    if notes is None:
        notes = []
    T = pd.read_csv(gnss_path)
    for name in [
        "Posix_Time",
        "posix_time",
        "time",
        "Time",
        "TIME",
        "gps_time",
        "GPSTime",
    ]:
        if name in T.columns:
            notes.append(f"GNSS: used '{name}' column")
            return T[name].to_numpy(dtype=float)
    notes.append("GNSS: no time column; assume 1 Hz")
    return np.arange(len(T), dtype=float)  # 1 Hz synthetic


def _truth_time(truth_path: str | None, notes: List[str] | None = None) -> np.ndarray | None:
    """Return truth time vector or ``None`` if ``truth_path`` is empty."""

    if not truth_path or not os.path.isfile(truth_path):
        return None
    if notes is None:
        notes = []
    try:
        T = pd.read_csv(truth_path, sep=None, engine="python")
    except Exception:
        T = pd.read_csv(truth_path, delim_whitespace=True, header=None)
    for name in [
        "time",
        "Time",
        "t",
        "T",
        "posix",
        "Posix_Time",
        "sec",
        "seconds",
    ]:
        if name in T.columns:
            notes.append(f"TRUTH: used '{name}' column")
            return T[name].to_numpy(dtype=float)
    col0 = T.iloc[:, 0].to_numpy(dtype=float)
    d = np.diff(col0)
    if np.all(np.isfinite(d)) and np.median(np.abs(d)) > 1e-5:
        notes.append("TRUTH: used column 0 as time")
        return col0
    notes.append("TRUTH: no time; assume 10 Hz synthetic")
    return np.arange(len(T), dtype=float) * 0.1


def print_timeline_summary(
    run_id: str,
    imu_path: str,
    gnss_path: str,
    truth_path: str | None,
    results_dir: str,
) -> str:
    """Print and save timeline summary.

    Parameters
    ----------
    run_id : str
        Identifier appended to output filenames.
    imu_path, gnss_path, truth_path : str
        Data file paths. ``truth_path`` may be ``None``.
    results_dir : str
        Directory in which ``*_timeline`` files are written.

    Returns
    -------
    str
        Path to the written ``*_timeline.txt`` file.
    """

    os.makedirs(results_dir, exist_ok=True)
    notes: List[str] = []

    t_imu = _detect_imu_time(imu_path, 0.0025, notes)
    t_gnss = _gnss_time(gnss_path, notes)
    t_truth = _truth_time(truth_path, notes) if truth_path else None

    s_imu = _timeline_stats(t_imu)
    s_gnss = _timeline_stats(t_gnss)
    s_tru = _timeline_stats(t_truth) if t_truth is not None else None

    def fmt(s, label):
        if s is None:
            return f"{label:<6}| (missing)"
        return (
            f"{label:<6}| n={s['n']:<7d}  hz={s['hz']:.6f}  dt_med={s['dt_med']:.6f}  "
            f"min/max dt=({s['dt_min']:.6f},{s['dt_max']:.6f})  "
            f"dur={s['dur']:.3f}s  t0={s['t0']:.6f}  t1={s['t1']:.6f}  "
            f"monotonic={'true' if s['monotonic'] else 'false'}"
        )

    header = f"== Timeline summary: {run_id} =="
    lines = [
        header,
        fmt(s_imu, "IMU"),
        fmt(s_gnss, "GNSS"),
        fmt(s_tru, "TRUTH"),
        "Notes:" if notes else "Notes: (none)",
    ]
    if notes:
        for n in notes:
            lines.append(f"- {n}")

    # print to console
    print("\n".join(lines))

    # write text file
    txt_path = os.path.join(results_dir, f"{run_id}_timeline.txt")
    with open(txt_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    # write json file
    json_path = os.path.join(results_dir, f"{run_id}_timeline.json")
    with open(json_path, "w") as f:
        json.dump(
            dict(run_id=run_id, imu=s_imu, gnss=s_gnss, truth=s_tru, notes=notes),
            f,
            indent=2,
        )

    return txt_path


__all__ = ["print_timeline_summary"]

