"""Input file readers for IMU, GNSS and truth datasets."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import numpy as np
import pandas as pd

from . import validators
from .validators import ensure_monotonic


def load_imu(path: str | Path, logger=None) -> Dict[str, np.ndarray]:
    """Load a minimal IMU ``.dat`` file."""
    p = Path(path)
    data = np.loadtxt(p, comments="#")
    t = data[:, 0]
    acc = data[:, 1:4]
    gyro = data[:, 4:7]
    # validators.assert_monotonic(t)
    t_fixed, dt_used, nfix = ensure_monotonic(t, logger)
    if nfix > 0:
        t = t_fixed
        # dt_est = dt_used
    validators.assert_no_nans("acc", acc)
    validators.assert_no_nans("gyro", gyro)
    hz = validators.assert_rate_stability(t)
    if logger:
        logger.debug("Loaded IMU %s (%d samples)", p, acc.shape[0])
    meta = {"source_path": str(p), "n": len(t), "hz_est": hz}
    return {"t": t, "acc": acc, "gyro": gyro, "columns": ["t", "ax", "ay", "az", "gx", "gy", "gz"], "meta": meta}


def load_gnss(path: str | Path, logger=None) -> Dict[str, np.ndarray]:
    """Load a GNSS CSV log."""
    p = Path(path)
    df = pd.read_csv(p)
    t = df["Posix_Time"].to_numpy(dtype=float)
    pos = df[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].to_numpy(dtype=float)
    vel = df[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].to_numpy(dtype=float)
    lat = df.get("Latitude_deg")
    lon = df.get("Longitude_deg")
    validators.assert_monotonic(t)
    validators.assert_no_nans("pos", pos)
    validators.assert_no_nans("vel", vel)
    hz = validators.assert_rate_stability(t)
    if logger:
        logger.debug("Loaded GNSS %s (%d samples)", p, pos.shape[0])
    meta = {"source_path": str(p), "n": len(t), "hz_est": hz}
    out = {"t": t, "ecef_pos": pos, "ecef_vel": vel, "columns": df.columns.tolist(), "meta": meta}
    if lat is not None and lon is not None:
        out["lat"] = lat.to_numpy(dtype=float)
        out["lon"] = lon.to_numpy(dtype=float)
    return out


def load_truth(path: str | Path, logger=None, timebase: str = "auto") -> Optional[Dict[str, np.ndarray]]:
    """Load truth state file if available."""
    if not path:
        return None
    p = Path(path)
    if not p.exists():
        if logger:
            logger.warning("Truth file %s missing", p)
        return None
    df = pd.read_csv(p, sep=r"\s+", engine="python", comment="#", header=None)
    t = pd.to_numeric(df.iloc[:, 1], errors="coerce").to_numpy(dtype=float)
    pos = df.iloc[:, 2:5].to_numpy(dtype=float)
    vel = df.iloc[:, 5:8].to_numpy(dtype=float)
    validators.assert_monotonic(t)
    validators.assert_no_nans("truth pos", pos)
    validators.assert_no_nans("truth vel", vel)
    hz = validators.assert_rate_stability(t)
    if logger:
        logger.debug("Loaded truth %s (%d samples)", p, pos.shape[0])
    meta = {"source_path": str(p), "n": len(t), "hz_est": hz, "timebase": timebase}
    return {"t": t, "ecef_pos": pos, "ecef_vel": vel, "columns": [], "meta": meta}


__all__ = ["load_imu", "load_gnss", "load_truth"]
