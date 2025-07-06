"""Physical constants used throughout the repository."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

DATA_ROOT = Path(__file__).resolve().parents[1]
STATE_FILE = DATA_ROOT / "STATE_X001.txt"


def _compute_local_gravity(state_file: Path) -> float:
    """Return gravity magnitude at the mean position of ``state_file``."""

    cols = [
        "count",
        "time",
        "X_ECEF_m",
        "Y_ECEF_m",
        "Z_ECEF_m",
        "VX_ECEF_mps",
        "VY_ECEF_mps",
        "VZ_ECEF_mps",
        "q0",
        "q1",
        "q2",
        "q3",
    ]
    df = pd.read_csv(state_file, sep="\s+", comment="#", names=cols)
    x, y, z = df[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].mean()

    transformer = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)
    lon, lat, alt = transformer.transform(x, y, z)
    lat_rad = np.deg2rad(lat)

    gamma_e = 9.7803253359
    k = 0.00193185265241
    e2 = 0.00669437999013
    sin2 = np.sin(lat_rad) ** 2
    g = gamma_e * (1 + k * sin2) / np.sqrt(1 - e2 * sin2)
    g -= 3.086e-6 * alt
    return float(g)


try:  # pragma: no cover - runtime check only
    GRAVITY = _compute_local_gravity(STATE_FILE)
except Exception:  # pragma: no cover - fallback
    GRAVITY = 9.81

EARTH_RATE = 7.2921e-5

