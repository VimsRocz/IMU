"""Project path helpers for data and results.

Centralizes locations for DATA and results so scripts avoid hardcoded
relative paths. Keep logic minimal and dependency‑free.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "DATA"
IMU_DIR = DATA_DIR / "IMU"
GNSS_DIR = DATA_DIR / "GNSS"
TRUTH_DIR = DATA_DIR / "Truth"
RESULTS_DIR = ROOT / "results"


def ensure_results_dir() -> Path:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    return RESULTS_DIR


def imu_path(name: str) -> Path:
    """Return full path for an IMU filename (e.g. 'IMU_X002.dat')."""
    p = Path(name)
    if p.is_file():
        return p
    return IMU_DIR / name


def gnss_path(name: str) -> Path:
    """Return full path for a GNSS filename (e.g. 'GNSS_X002.csv')."""
    p = Path(name)
    if p.is_file():
        return p
    return GNSS_DIR / name


def truth_path(name: str = "STATE_X001.txt") -> Path:
    """Return full path for the truth file under DATA/Truth."""
    p = Path(name)
    if p.is_file():
        return p
    return TRUTH_DIR / name


def normalize_gnss_headers(df):
    """Normalize known header variants in a GNSS DataFrame in-place.

    - 'Height_deg' -> 'Height_m'
    Returns the DataFrame for convenience.
    """
    if "Height_deg" in df.columns and "Height_m" not in df.columns:
        df = df.rename(columns={"Height_deg": "Height_m"})
    return df

