from __future__ import annotations

from pathlib import Path
import re


def data_root() -> Path:
    """Return path to shared DATA directory."""
    return Path(__file__).resolve().parents[3] / "DATA"


def outputs_root() -> Path:
    """Return path to OUTPUTS directory for Python stack."""
    return Path(__file__).resolve().parents[2] / "OUTPUTS"


def infer_run_name(gnss_path: str | Path, imu_path: str | Path) -> str:
    """Infer run name from file names containing an X### token."""
    pattern = re.compile(r"X(\d+)")
    for p in [gnss_path, imu_path]:
        m = pattern.search(str(p))
        if m:
            return f"X{m.group(1)}"
    return "RUN"
