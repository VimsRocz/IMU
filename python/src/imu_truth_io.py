"""Robust truth-table reader for IMU/GNSS datasets.

Usage:
    from imu_truth_io import load_truth
    tbl = load_truth("path/to/STATE_X001.txt")

The parser tolerates comment headers, variable delimiters (whitespace or commas),
and auto-detects column names.
"""

from __future__ import annotations
from pathlib import Path
import re
import numpy as np


class TruthTable:
    """Lightweight container with named columns and convenience getters."""

    def __init__(
        self,
        t,
        x=None,
        y=None,
        z=None,
        vx=None,
        vy=None,
        vz=None,
        q0=None,
        q1=None,
        q2=None,
        q3=None,
        raw=None,
        cols=None,
    ):
        self.t = t
        self.x, self.y, self.z = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz
        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3
        self.raw = raw
        self.cols = cols or {}

    def has_ecef(self) -> bool:
        """Return True if X/Y/Z columns are present."""

        return self.x is not None and self.y is not None and self.z is not None


def _detect_delim(sample_line: str) -> str | None:
    """Return ',' if commas present; otherwise None for whitespace."""

    return "," if "," in sample_line else None


def _strip_comments(lines):
    for ln in lines:
        yield ln.rstrip("\n")


def _parse_header_names(lines):
    for ln in lines:
        if ln.strip().startswith("#"):
            hdr = re.sub(r"\s+", " ", ln.strip()[1:].strip())
            if hdr:
                return [c.strip() for c in hdr.split(" ")]
    return None


def _lower_names(names):
    try:
        return [str(n).strip().lower() for n in names]
    except Exception:
        return None


def _col_index(names_lc, key_candidates):
    for k in key_candidates:
        if k in names_lc:
            return names_lc.index(k)
    return None


def load_truth(path) -> TruthTable:
    """Return a :class:`TruthTable` parsed from *path*.

    The function tolerates comment headers and autodetects delimiters. Missing
    columns are left as ``None``.
    """

    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Truth file not found: {p}")

    with p.open("r", encoding="utf-8", errors="ignore") as f:
        sample = [next(f) for _ in range(50) if not f.closed]
    sample_no_blanks = [s for s in sample if s.strip()]
    if not sample_no_blanks:
        raise ValueError("Truth file is empty")

    delim = _detect_delim("".join(sample_no_blanks))
    hdr_names = _parse_header_names(sample_no_blanks)
    names_lc = _lower_names(hdr_names) if hdr_names else None

    try:
        data = np.loadtxt(p, comments="#", delimiter=delim, ndmin=2)
    except Exception:
        data = np.loadtxt(p, comments="#", ndmin=2)

    if data.ndim == 1:
        data = data[None, :]

    cols = {}
    if names_lc:
        maps = {
            "t": ["t", "time", "timestamp", "sec", "s"],
            "x": ["x_ecef_m", "x", "xecef", "xecef_m"],
            "y": ["y_ecef_m", "y", "yecef", "yecef_m"],
            "z": ["z_ecef_m", "z", "zecef", "zecef_m"],
            "vx": ["vx_ecef_mps", "vx", "vxecef", "vxecef_mps"],
            "vy": ["vy_ecef_mps", "vy", "vyecef", "vyecef_mps"],
            "vz": ["vz_ecef_mps", "vz", "vzecef", "vzecef_mps"],
            "q0": ["q0", "qw", "w"],
            "q1": ["q1", "qx", "xq"],
            "q2": ["q2", "qy", "yq"],
            "q3": ["q3", "qz", "zq"],
        }
        for key, cand in maps.items():
            idx = _col_index(names_lc, cand)
            if idx is not None and idx < data.shape[1]:
                cols[key] = idx
    else:
        assumed = [
            "count",
            "t",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "q0",
            "q1",
            "q2",
            "q3",
        ]
        cols = {k: i for i, k in enumerate(assumed) if i < data.shape[1]}

    def pick(name):
        i = cols.get(name)
        return data[:, i] if i is not None else None

    t = pick("t")
    if t is None:
        t = data[:, 1] if data.shape[1] > 1 else np.arange(data.shape[0], dtype=float)

    if len(t) < 2:
        raise ValueError("Truth table has too few rows")
    if not np.all(np.isfinite(t)):
        raise ValueError("Truth time contains non-finite values")

    return TruthTable(
        t=t.astype(float),
        x=pick("x"),
        y=pick("y"),
        z=pick("z"),
        vx=pick("vx"),
        vy=pick("vy"),
        vz=pick("vz"),
        q0=pick("q0"),
        q1=pick("q1"),
        q2=pick("q2"),
        q3=pick("q3"),
        raw=data,
        cols=cols,
    )
