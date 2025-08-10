from __future__ import annotations

"""Summarise timeline and sampling rates for IMU, GNSS, and truth files.

Usage:
    from tools.inspect_timing import describe_files, table_for_terminal
    rows = describe_files('IMU_X002.dat', 'GNSS_X002.csv', 'STATE_X001.txt')
    print(table_for_terminal(rows))
"""

import json
import math
from pathlib import Path
from dataclasses import dataclass, asdict

import numpy as np
import pandas as pd

# ---------- helpers ----------
def _median_dt(t: np.ndarray) -> float:
    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    if dt.size == 0:
        return float("nan")
    return float(np.median(dt))


def _rate(dt: float) -> float:
    if not math.isfinite(dt) or dt <= 0:
        return float("nan")
    return 1.0 / dt


def _fmt_utc(ts: float | None) -> str:
    if ts is None or not math.isfinite(ts):
        return "—"
    # don’t import datetime just to avoid tz drama; keep POSIX float
    return f"{ts:.3f} (POSIX s)"


@dataclass
class TimeSummary:
    source: str                 # 'IMU', 'GNSS', 'TRUTH'
    samples: int
    t_start: float | None       # POSIX if known else relative 0.0
    t_end: float | None
    duration_s: float | None
    median_dt_s: float | None
    rate_hz: float | None
    absolute_epoch: bool        # True if t_start is absolute (POSIX)
    notes: str = ""

    def to_row(self):
        return [
            self.source,
            self.samples,
            f"{self.duration_s:.3f}" if self.duration_s is not None else "—",
            f"{self.median_dt_s:.6f}" if self.median_dt_s is not None else "—",
            f"{self.rate_hz:.3f}" if self.rate_hz is not None else "—",
            _fmt_utc(self.t_start),
            _fmt_utc(self.t_end),
            "ABS" if self.absolute_epoch else "REL",
            self.notes or "",
        ]


def _unwrap_reset_seconds(t: np.ndarray) -> np.ndarray:
    """Handle per-second resetting fractional clocks (e.g., 0..1 ramp)."""
    if t.size < 2:
        return t.copy()
    unwrap = t.copy().astype(float)
    jumps = np.diff(unwrap)
    carry = 0.0
    out = np.empty_like(unwrap)
    out[0] = unwrap[0]
    for i, d in enumerate(jumps, start=1):
        if d < -0.5:
            carry += 1.0
        out[i] = unwrap[i] + carry
    out -= out[0]
    return out


# ---------- readers ----------
def read_gnss_time(csv_path: Path) -> tuple[np.ndarray, bool, str]:
    """Return (t, absolute_epoch, note) for a GNSS CSV."""
    df = pd.read_csv(csv_path)
    note = ""
    if "Posix_Time" in df.columns:
        t = df["Posix_Time"].to_numpy(dtype=float)
        return t, True, note
    utc_cols = ["UTC_yyyy", "UTC_MM", "UTC_dd", "UTC_HH", "UTC_mm", "UTC_ss"]
    if all(c in df.columns for c in utc_cols):
        y, mo, d, hh, mm = (df[c].to_numpy(int) for c in utc_cols[:5])
        ss = df["UTC_ss"].to_numpy(float)
        ts = pd.to_datetime(
            dict(year=y, month=mo, day=d, hour=hh, minute=mm, second=np.floor(ss).astype(int)),
            utc=True,
            errors="coerce",
        ).astype("int64") / 1e9
        frac = ss - np.floor(ss)
        t = ts + frac
        note = "POSIX built from UTC columns"
        return t.to_numpy(), True, note
    note = "No POSIX/UTC; using sample index"
    return np.arange(len(df), dtype=float), False, note


def read_truth_time(path: Path) -> tuple[np.ndarray, bool, str]:
    """Return (t, absolute_epoch, note) for a truth STATE_*.txt file."""
    try:
        df = pd.read_csv(path, sep=None, engine="python")
    except Exception:
        df = pd.read_table(path, sep=None, engine="python")
    note = ""
    cand_cols = [c for c in df.columns if c.lower() in ("t", "time", "timestamp", "posix_time", "sec")]
    if cand_cols:
        s = df[cand_cols[0]].to_numpy(dtype=float)
    else:
        s = df.iloc[:, 0].to_numpy(dtype=float)
    absolute = np.nanmax(s) > 1e8  # crude POSIX heuristic
    return s, absolute, note


def read_imu_time(dat_path: Path) -> tuple[np.ndarray, bool, str]:
    """Return (t, absolute_epoch, note) for an IMU .dat file."""
    try:
        df = pd.read_csv(dat_path, sep=None, engine="python", on_bad_lines="skip")
    except Exception:
        df = pd.read_table(dat_path, sep=None, engine="python", header=None, on_bad_lines="skip")

    for col in ("time", "t", "Time", "sec", "seconds", "timestamp", "TIME"):
        if col in df.columns:
            t = df[col].to_numpy(dtype=float)
            t_u = _unwrap_reset_seconds(t)
            return t_u, False, "IMU time unwrapped (relative)"
    t = df.iloc[:, 0].to_numpy(dtype=float)
    t_u = _unwrap_reset_seconds(t)
    return t_u, False, "IMU time unwrapped (relative)"


# ---------- public API ----------
def describe_files(imu_path: str, gnss_path: str, truth_path: str | None = None):
    """Describe timelines for IMU, GNSS, and optional truth files."""
    rows: list[TimeSummary] = []

    t_gnss, abs_gnss, note_g = read_gnss_time(Path(gnss_path))
    med_g = _median_dt(t_gnss)
    rows.append(
        TimeSummary(
            source="GNSS",
            samples=t_gnss.size,
            t_start=float(t_gnss[0]) if t_gnss.size else None,
            t_end=float(t_gnss[-1]) if t_gnss.size else None,
            duration_s=float(t_gnss[-1] - t_gnss[0]) if t_gnss.size > 1 else None,
            median_dt_s=med_g,
            rate_hz=_rate(med_g),
            absolute_epoch=abs_gnss,
            notes=note_g,
        )
    )

    t_imu, abs_imu, note_i = read_imu_time(Path(imu_path))
    med_i = _median_dt(t_imu)
    rows.append(
        TimeSummary(
            source="IMU",
            samples=t_imu.size,
            t_start=float(t_imu[0]) if t_imu.size else None,
            t_end=float(t_imu[-1]) if t_imu.size else None,
            duration_s=float(t_imu[-1] - t_imu[0]) if t_imu.size > 1 else None,
            median_dt_s=med_i,
            rate_hz=_rate(med_i),
            absolute_epoch=abs_imu,
            notes=note_i,
        )
    )

    if truth_path and Path(truth_path).exists():
        t_true, abs_true, note_t = read_truth_time(Path(truth_path))
        med_t = _median_dt(t_true)
        rows.append(
            TimeSummary(
                source="TRUTH",
                samples=t_true.size,
                t_start=float(t_true[0]) if t_true.size else None,
                t_end=float(t_true[-1]) if t_true.size else None,
                duration_s=float(t_true[-1] - t_true[0]) if t_true.size > 1 else None,
                median_dt_s=med_t,
                rate_hz=_rate(med_t),
                absolute_epoch=abs_true,
                notes=note_t,
            )
        )

    return rows


def table_for_terminal(rows: list[TimeSummary]) -> str:
    """Return a simple monospaced table for printing to the terminal."""
    headers = [
        "Source",
        "Samples",
        "Duration [s]",
        "Δt_med [s]",
        "Rate [Hz]",
        "Start",
        "End",
        "Epoch",
        "Notes",
    ]
    data = [r.to_row() for r in rows]
    widths = [max(len(str(x)) for x in col) for col in zip(headers, *data)]
    def fmt_row(r):
        return "  ".join(str(c).ljust(w) for c, w in zip(r, widths))
    return "\n".join([fmt_row(headers)] + [fmt_row(d) for d in data])


def write_reports(rows: list[TimeSummary], out_json: Path, out_md: Path):
    """Write summary rows to JSON and Markdown outputs."""
    out_json.write_text(json.dumps([asdict(r) for r in rows], indent=2))
    md = ["# Timeline & Sampling Summary", ""]
    md.append("| Source | Samples | Duration [s] | Δt_med [s] | Rate [Hz] | Start | End | Epoch | Notes |")
    md.append("|---|---:|---:|---:|---:|---|---|:--:|---|")
    for r in rows:
        md.append(
            "| {s} | {n} | {d} | {dt} | {hz} | {ts} | {te} | {ep} | {notes} |".format(
                s=r.source,
                n=r.samples,
                d=f"{r.duration_s:.3f}" if r.duration_s is not None else "—",
                dt=f"{r.median_dt_s:.6f}" if r.median_dt_s is not None else "—",
                hz=f"{r.rate_hz:.3f}" if r.rate_hz is not None else "—",
                ts=_fmt_utc(r.t_start),
                te=_fmt_utc(r.t_end),
                ep="ABS" if r.absolute_epoch else "REL",
                notes=r.notes.replace("|", "/"),
            )
        )
    out_md.write_text("\n".join(md))
