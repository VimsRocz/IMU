"""Dataset timeline helpers.

This module prints and optionally saves concise timing summaries for IMU,
GNSS and optional truth files. A MATLAB counterpart lives in
``MATLAB/src/utils/timeline_summary.m``.
"""

from __future__ import annotations

from pathlib import Path
import numpy as np
import pandas as pd


def _read_truth_time(truth_path, notes):
    """Read STATE_* truth file robustly:

    - Ignore lines starting with '#'
    - Split on any whitespace
    - Coerce 1st column to numeric, drop NaN rows
    - Return time starting at zero
    """
    if not truth_path or not Path(truth_path).exists():
        return None

    st = pd.read_csv(
        truth_path,
        sep=r"\s+",
        engine="python",
        header=None,
        comment="#",
        na_values=["NaN", "nan", "INF", "-INF", "inf", "-inf", ""],
        keep_default_na=True,
    )

    # coerce first column to numeric; drop bad rows
    t = pd.to_numeric(st.iloc[:, 0], errors="coerce").to_numpy(np.float64)
    mask = np.isfinite(t)
    t = t[mask]
    if t.size < 2:
        notes.append(
            "TRUTH: failed to parse time column; insufficient numeric rows."
        )
        return None

    # normalize to start at zero
    t = t - t[0]
    return t


def _read_imu_numeric(path: str | Path) -> pd.DataFrame:
    """Read an IMU ``.dat`` file coercing all columns to numeric.

    Any non-numeric tokens become ``NaN`` which keeps ``np.isfinite`` safe.
    """

    df = pd.read_csv(
        path,
        sep=r"\s+",
        engine="python",
        header=None,
        comment="#",
        na_values=["NaN", "nan", "INF", "-INF", "inf", "-inf", ""],
        keep_default_na=True,
    )
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(how="all")
    return df


def _unwrap_seconds(subsec: np.ndarray, dt_hint: float = 0.0025) -> np.ndarray:
    """Convert fractional seconds that reset each second into a uniform grid."""

    subsec = np.asarray(subsec, dtype=np.float64)
    d = np.diff(subsec)
    wrap = d < -0.5
    step = np.concatenate([[0.0], np.cumsum(wrap.astype(float))])
    t = subsec + step
    t -= t[0]
    n = len(t)
    return np.arange(n, dtype=np.float64) * float(dt_hint)


def _detect_imu_time(imu_path: str | Path, dt_hint: float, notes: list[str]) -> np.ndarray:
    """Detect or construct a monotonic IMU time vector.

    The algorithm tries each column as a fractional-second counter or a
    strictly monotonic column. If all fail it falls back to a uniform grid.
    """

    df = _read_imu_numeric(imu_path)
    n = len(df)
    best_t: np.ndarray | None = None
    best_score = np.inf

    for c in df.columns:
        col = df[c].to_numpy(dtype=np.float64)
        finite = np.isfinite(col)
        if finite.mean() < 0.9:
            continue

        cmin = np.nanmin(col)
        cmax = np.nanmax(col)
        if (cmin >= -1e-3) and (cmax <= 1.001):
            t = _unwrap_seconds(col, dt_hint)
            dt = np.diff(t)
            score = abs(np.median(dt) - dt_hint)
            if np.all(dt > 0) and score < best_score:
                best_t = t
                best_score = score

        d = np.diff(col[finite])
        if d.size > 0 and np.all(d > 0):
            med = float(np.median(d))
            if np.isfinite(med) and med > 0:
                score = abs(med - dt_hint)
                t = col - np.nanmin(col)
                if score < best_score:
                    best_t = t
                    best_score = score

    if best_t is None:
        notes.append("IMU: could not detect a valid time column; using uniform grid via dt_hint.")
        best_t = np.arange(n, dtype=np.float64) * float(dt_hint)
    else:
        notes.append("IMU: detected/constructed time column successfully.")

    return np.asarray(best_t, dtype=np.float64)


def print_timeline_summary(
    run_id: str,
    imu_path: str | Path,
    gnss_path: str | Path,
    truth_path: str | Path | None = None,
    out_dir: str | Path | None = None,
) -> str | None:
    """Print dataset timeline summary and optionally save to ``out_dir``."""

    notes: list[str] = []
    tt: np.ndarray | None = None
    tdt: np.ndarray | None = None
    thz: float | None = None

    # --- IMU ---
    t_imu = _detect_imu_time(imu_path, 0.0025, notes)
    imu_dt = np.diff(t_imu)
    imu_hz = 1.0 / np.median(imu_dt)
    print(f"== Timeline summary: {run_id} ==")
    print(
        "IMU   | n={:d}   hz={:.6f}  dt_med={:.6f}  min/max dt=({:.6f},{:.6f})  "
        "dur={:.3f}s  t0={:.6f}  t1={:.6f}  monotonic={}".format(
            len(t_imu),
            imu_hz,
            np.median(imu_dt),
            np.min(imu_dt),
            np.max(imu_dt),
            t_imu[-1] - t_imu[0],
            t_imu[0],
            t_imu[-1],
            bool(np.all(imu_dt > 0)),
        )
    )

    # --- GNSS ---
    g = pd.read_csv(gnss_path)
    tg = g["Posix_Time"].to_numpy(np.float64)
    tg = tg - tg[0]
    gdt = np.diff(tg)
    ghz = 1.0 / np.median(gdt)
    print(
        "GNSS  | n={:d}     hz={:.6f}  dt_med={:.6f}  min/max dt=({:.6f},{:.6f})  "
        "dur={:.3f}s  t0={:.6f}  t1={:.6f}  monotonic={}".format(
            len(tg),
            ghz,
            np.median(gdt),
            np.min(gdt),
            np.max(gdt),
            tg[-1] - tg[0],
            tg[0],
            tg[-1],
            bool(np.all(gdt > 0)),
        )
    )

    # --- TRUTH (optional) ---
    if truth_path and Path(truth_path).exists():
        tt = _read_truth_time(truth_path, notes)
        if tt is not None:
            tdt = np.diff(tt)
            thz = 1.0 / np.median(tdt)
            print(
                "TRUTH | n={:d}    hz={:.6f}  dt_med={:.6f}  min/max dt=({:.6f},{:.6f})  "
                "dur={:.3f}s  t0={:.6f}  t1={:.6f}  monotonic={}".format(
                    len(tt),
                    thz,
                    np.median(tdt),
                    np.min(tdt),
                    np.max(tdt),
                    tt[-1] - tt[0],
                    tt[0],
                    tt[-1],
                    bool(np.all(tdt > 0)),
                )
            )
        else:
            print("TRUTH | present but unreadable (see Notes).")
    else:
        print("TRUTH | (not provided)")

    if notes:
        print("Notes:")
        for n in notes:
            print(f"- {n}")

    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        out = Path(out_dir) / f"{run_id}_timeline.txt"
        with open(out, "w", encoding="utf-8") as f:
            f.write(f"== Timeline summary: {run_id} ==\n")
            f.write(
                "IMU   | n={:d}   hz={:.6f}  dt_med={:.6f}  min/max dt=({:.6f},{:.6f})  "
                "dur={:.3f}s  t0={:.6f}  t1={:.6f}  monotonic={}".format(
                    len(t_imu),
                    imu_hz,
                    np.median(imu_dt),
                    np.min(imu_dt),
                    np.max(imu_dt),
                    t_imu[-1] - t_imu[0],
                    t_imu[0],
                    t_imu[-1],
                    bool(np.all(imu_dt > 0)),
                )
            )
            f.write("\n")
            f.write(
                "GNSS  | n={:d}     hz={:.6f}  dt_med={:.6f}  min/max dt=({:.6f},{:.6f})  "
                "dur={:.3f}s  t0={:.6f}  t1={:.6f}  monotonic={}".format(
                    len(tg),
                    ghz,
                    np.median(gdt),
                    np.min(gdt),
                    np.max(gdt),
                    tg[-1] - tg[0],
                    tg[0],
                    tg[-1],
                    bool(np.all(gdt > 0)),
                )
            )
            f.write("\n")
            if tt is not None:
                f.write(
                    "TRUTH | n={:d}    hz={:.6f}  dt_med={:.6f}  min/max dt=({:.6f},{:.6f})  "
                    "dur={:.3f}s  t0={:.6f}  t1={:.6f}  monotonic={}".format(
                        len(tt),
                        thz,
                        np.median(tdt),
                        np.min(tdt),
                        np.max(tdt),
                        tt[-1] - tt[0],
                        tt[0],
                        tt[-1],
                        bool(np.all(tdt > 0)),
                    )
                )
                f.write("\n")
            elif truth_path and Path(truth_path).exists():
                f.write("TRUTH | present but unreadable (see Notes).\n")
            else:
                f.write("TRUTH | (not provided)\n")
            if notes:
                f.write("Notes:\n")
                for n in notes:
                    f.write(f"- {n}\n")
        return str(out)

    return None


# Backward compatibility with existing callers
print_timeline = print_timeline_summary


__all__ = ["print_timeline_summary", "print_timeline"]

