"""Dataset timeline helpers.

This module prints and optionally saves concise timing summaries for IMU,
GNSS and optional truth files. A MATLAB counterpart lives in
``MATLAB/src/utils/timeline_summary.m``.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
import pandas as pd

def _unwrap_seconds(subsec: np.ndarray, dt_hint: float | None = None) -> np.ndarray:
    """Unwrap a fractional-second counter that resets each second."""
    subsec = np.asarray(subsec, float)
    d = np.diff(subsec)
    wrap = d < -0.5
    step = np.concatenate([[0], np.cumsum(wrap)])
    t = subsec + step
    t -= t[0]
    if dt_hint is None:
        dgood = np.diff(subsec)
        m = np.median(dgood[(np.abs(dgood)>0) & (np.abs(dgood)<0.5)])
        if not np.isfinite(m) or m <= 0:
            dt_hint = 0.0025
        else:
            dt_hint = m
    n = len(t)
    return np.arange(n) * dt_hint

def print_timeline(run_id: str, imu_path: str, gnss_path: str,
                   truth_path: str | None = None, out_dir: str | None = None) -> None:
    """Print dataset timeline summary and optionally save to ``out_dir``.

    Parameters
    ----------
    run_id : str
        Identifier for header and filenames.
    imu_path, gnss_path : str
        Data file paths.
    truth_path : str, optional
        Optional truth file path.
    out_dir : str, optional
        Directory to save ``*_timeline.txt``.
    """
    print(f"== Timeline summary: {run_id} ==")
    # IMU
    imu = pd.read_csv(imu_path, delim_whitespace=True, header=None, engine="python")
    t_raw = imu.iloc[:,1].to_numpy()
    d = np.diff(t_raw)
    if np.any(d < -0.5) or np.any(d > 0.5):
        t_imu = _unwrap_seconds(t_raw, 0.0025)
    else:
        t_imu = t_raw - t_raw[0]
    dt = np.diff(t_imu)
    hz = 1/np.median(dt)
    line_imu = ("IMU    | n={n}  hz={hz:.6f}  dt_med={dtmed:.6f}  "
                "min/max dt=({dtmin:.6f},{dtmax:.6f})  dur={dur:.3f}s  "
                "t0={t0:.6f}  t1={t1:.6f}  monotonic={mono}").format(
                    n=len(t_imu), hz=hz, dtmed=np.median(dt),
                    dtmin=dt.min(), dtmax=dt.max(), dur=t_imu[-1]-t_imu[0],
                    t0=t_imu[0], t1=t_imu[-1], mono=np.all(dt>0))
    print(line_imu)
    # GNSS
    g = pd.read_csv(gnss_path)
    tg = g["Posix_Time"].to_numpy()
    tg = tg - tg[0]
    d = np.diff(tg)
    hz = 1/np.median(d)
    line_gnss = ("GNSS   | n={n}    hz={hz:.6f}  dt_med={dtmed:.6f}  "
                 "min/max dt=({dtmin:.6f},{dtmax:.6f})  dur={dur:.3f}s  "
                 "t0={t0:.6f}  t1={t1:.6f}  monotonic={mono}").format(
                     n=len(tg), hz=hz, dtmed=np.median(d), dtmin=d.min(),
                     dtmax=d.max(), dur=tg[-1]-tg[0], t0=tg[0], t1=tg[-1],
                     mono=np.all(d>0))
    print(line_gnss)
    # TRUTH (optional)
    line_truth = "TRUTH  | (not provided)"
    if truth_path and Path(truth_path).exists():
        st = pd.read_csv(truth_path, delim_whitespace=True, header=None)
        tt = st.iloc[:,0].to_numpy()
        tt = tt - tt[0]
        d = np.diff(tt)
        hz = 1/np.median(d)
        line_truth = ("TRUTH  | n={n}   hz={hz:.6f}  dt_med={dtmed:.6f}  "
                      "min/max dt=({dtmin:.6f},{dtmax:.6f})  dur={dur:.3f}s  "
                      "t0={t0:.6f}  t1={t1:.6f}  monotonic={mono}").format(
                          n=len(tt), hz=hz, dtmed=np.median(d),
                          dtmin=d.min(), dtmax=d.max(), dur=tt[-1]-tt[0],
                          t0=tt[0], t1=tt[-1], mono=np.all(d>0))
        print(line_truth)
    else:
        print(line_truth)
    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        out_path = Path(out_dir) / f"{run_id}_timeline.txt"
        with out_path.open("w", encoding="utf-8") as f:
            f.write(f"== Timeline summary: {run_id} ==\n")
            f.write(line_imu + "\n")
            f.write(line_gnss + "\n")
            f.write(line_truth + "\n")
        print(f"[DATA TIMELINE] Saved {out_path}")
