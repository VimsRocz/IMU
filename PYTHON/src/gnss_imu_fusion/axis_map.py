import numpy as np
from typing import Optional, Tuple


def default_C_bs() -> np.ndarray:
    """Return default sensor→body DCM.

    Assumes body axes use the NED convention (X-forward, Y-right, Z-down)
    while the sensor appears to have X aligned mostly with down and Z with
    forward in the provided dataset. A 90° rotation about +Y maps:
      - sensor X → -body Z
      - sensor Z →  body X
      - sensor Y →  body Y
    """
    return np.array([[0.0, 0.0, 1.0],
                     [0.0, 1.0, 0.0],
                     [-1.0, 0.0, 0.0]], dtype=float)


def apply_axis_map(accel_s: np.ndarray,
                   gyro_s: np.ndarray,
                   C_bs: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Map sensor-frame accel/gyro to body frame using ``C_bs``.

    Parameters
    ----------
    accel_s, gyro_s : array-like, shape (N,3)
        Specific force and angular rate in the sensor frame.
    C_bs : array-like, optional
        DCM mapping sensor→body. If ``None``, uses :func:`default_C_bs`.
    """
    C = default_C_bs() if C_bs is None else np.asarray(C_bs, dtype=float)
    return (accel_s @ C.T, gyro_s @ C.T)


def sanity_check_tilt(g_b: np.ndarray) -> str:
    """Return a short message about gravity magnitude and tilt from body Z.

    Expects a mean specific-force vector during rest in the body frame.
    """
    g = float(np.linalg.norm(g_b))
    if not np.isfinite(g) or g <= 0:
        return "WARN: gravity magnitude not finite"
    tilt_cos = np.clip(g_b[2] / g, -1.0, 1.0)
    tilt_deg = float(np.degrees(np.arccos(tilt_cos)))
    if g < 9.0 or g > 10.1:
        return f"WARN: |g| unexpected: {g:.3f} m/s²; tilt(Z): {tilt_deg:.2f}°"
    return f"Tilt from body Z: {tilt_deg:.2f}° (expect small at rest)"
