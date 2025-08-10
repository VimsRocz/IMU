"""Convenience constants for repository paths."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "DATA"
OUT = ROOT / "python" / "results"

GNSS = DATA / "GNSS"
IMU = DATA / "IMU"
TRUTH = DATA / "TRUTH"

OUT.mkdir(parents=True, exist_ok=True)
