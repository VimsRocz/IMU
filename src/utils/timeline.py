"""Utility functions for dataset timeline summaries.

Usage:
    from timeline import summarize_files

This module provides helpers to derive sample rates, detect sub-second
clock resets in IMU logs, and export human-friendly timeline reports.
A MATLAB counterpart is implemented in ``summarize_timeline.m`` for
feature parity.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pandas as pd


def _rate_from_times(t: np.ndarray) -> dict:
    """Return timing statistics for a time vector.

    Parameters
    ----------
    t : array-like
        Time stamps in seconds.

    Returns
    -------
    dict
        Mapping with keys ``n``, ``hz``, ``dt_med``, ``dt_min``, ``dt_max``,
        ``duration``, ``t0``, ``t1`` and ``monotonic``. Values may be ``None``
        when insufficient samples are available.
    """

    t = np.asarray(t, dtype=float)
    if t.size < 2:
        return {
            "n": int(t.size),
            "hz": None,
            "dt_med": None,
            "dt_min": None,
            "dt_max": None,
            "duration": 0.0,
            "t0": None,
            "t1": None,
            "monotonic": True,
        }
    dt = np.diff(t)
    monotonic = np.all(dt >= -1e-9)
    if t[-1] >= t[0]:
        duration = float(t[-1] - t[0])
    else:
        duration = float((t[-1] + (t.size - 1) * np.median(dt)) - t[0])
    dt_med = float(np.median(dt))
    hz = float(1.0 / dt_med) if dt_med > 0 else None
    return {
        "n": int(t.size),
        "hz": hz,
        "dt_med": dt_med,
        "dt_min": float(np.min(dt)),
        "dt_max": float(np.max(dt)),
        "duration": duration,
        "t0": float(t[0]),
        "t1": float(t[-1]),
        "monotonic": bool(monotonic),
    }


def _unwrap_imu_seconds(t_subsec: np.ndarray) -> np.ndarray:
    """Unwrap an IMU clock that resets each second.

    Parameters
    ----------
    t_subsec : array-like
        Sub-second time stamps in the range ``[0, 1)``.

    Returns
    -------
    np.ndarray
        Monotonic time vector with one-second rollover removed.
    """

    t = np.asarray(t_subsec, dtype=float)
    if t.size == 0:
        return t
    out = np.zeros_like(t)
    acc = 0.0
    out[0] = t[0]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt < -0.5:  # big negative jump => new second started
            acc += 1.0
        out[i] = t[i] + acc
    return out


def summarize_files(
    imu_path: str | os.PathLike,
    gnss_path: str | os.PathLike,
    truth_path: str | os.PathLike | None = None,
    out_dir: str | os.PathLike = "results",
    run_id: str | None = None,
):
    """Summarize IMU, GNSS and optional truth files and save reports.

    Parameters
    ----------
    imu_path, gnss_path, truth_path : path-like
        Input data files. ``truth_path`` may be ``None``.
    out_dir : str or Path, optional
        Output directory for ``*_timeline.json`` and ``*_timeline.txt``.
    run_id : str, optional
        Identifier used in file names.

    Returns
    -------
    tuple
        ``(summary, json_path, txt_path)`` where ``summary`` is a dictionary
        containing per-file timing info.
    """

    os.makedirs(out_dir, exist_ok=True)
    run_id = run_id or "run"

    summary: dict[str, object] = {"run_id": run_id, "notes": []}

    # ---- GNSS (CSV with Posix_Time if available) ----
    gnss = pd.read_csv(gnss_path)
    if "Posix_Time" in gnss.columns:
        t_gnss = gnss["Posix_Time"].to_numpy()
    else:
        t_gnss = np.arange(len(gnss), dtype=float)
        summary["notes"].append("GNSS: Posix_Time not found; assumed 1 Hz via row index.")
    summary["gnss"] = {"file": os.path.basename(gnss_path), **_rate_from_times(t_gnss)}

    # ---- IMU (DAT) ----
    imu = pd.read_csv(imu_path, delim_whitespace=True, header=None, comment="#", engine="python")
    candidate_time = None
    for col in range(min(imu.shape[1], 6)):
        v = imu[col].astype(float).to_numpy()
        if np.isfinite(v).all() and np.ptp(v) > 0 and np.ptp(v) < 1e6:
            dv = np.diff(v)
            if v.size > 10 and np.median(np.abs(dv)) > 1e-4 and np.median(np.abs(dv)) < 1.0:
                candidate_time = v
                break
    if candidate_time is None:
        subsec = None
        for col in range(imu.shape[1] - 1, max(-1, imu.shape[1] - 4), -1):
            v = imu[col].astype(float).to_numpy()
            if np.all((v >= 0) & (v < 1.0)):
                subsec = v
                break
        if subsec is not None:
            t_imu = _unwrap_imu_seconds(subsec)
        else:
            dt_guess = 0.0025
            t_imu = np.arange(len(imu), dtype=float) * dt_guess
            summary["notes"].append("IMU: no time column found; using 400 Hz fallback (dt=0.0025s).")
    else:
        if np.all((candidate_time >= 0) & (candidate_time < 1.0)):
            t_imu = _unwrap_imu_seconds(candidate_time)
        else:
            t_imu = candidate_time
    summary["imu"] = {"file": os.path.basename(imu_path), **_rate_from_times(t_imu)}

    # ---- Truth (STATE_*.txt) ----
    if truth_path and os.path.isfile(truth_path):
        truth = pd.read_csv(
            truth_path,
            delim_whitespace=True,
            comment="#",
            engine="python",
            names=[
                "idx",
                "time",
                "X",
                "Y",
                "Z",
                "VX",
                "VY",
                "VZ",
                "q0",
                "q1",
                "q2",
                "q3",
            ],
        )
        t_truth = truth["time"].to_numpy()
        summary["truth"] = {"file": os.path.basename(truth_path), **_rate_from_times(t_truth)}
    else:
        summary["truth"] = {"file": None}

    json_path = os.path.join(out_dir, f"{run_id}_timeline.json")
    txt_path = os.path.join(out_dir, f"{run_id}_timeline.txt")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    with open(txt_path, "w", encoding="utf-8") as f:
        def line(tag: str, s: dict) -> None:
            f.write(
                f"{tag:6s} | n={s.get('n')}  hz={s.get('hz'):.6f}  dt_med={s.get('dt_med'):.6f}  "
                f"min/max dt=({s.get('dt_min'):.6f},{s.get('dt_max'):.6f})  "
                f"dur={s.get('duration'):.3f}s  t0={s.get('t0')}  t1={s.get('t1')}  "
                f"monotonic={s.get('monotonic')}\n"
            )

        f.write(f"== Timeline summary: {run_id} ==\n")
        line("IMU", summary["imu"])
        line("GNSS", summary["gnss"])
        if summary["truth"]["file"]:
            line("TRUTH", summary["truth"])
        if summary["notes"]:
            f.write("\nNotes:\n- " + "\n- ".join(summary["notes"]) + "\n")
    return summary, json_path, txt_path
