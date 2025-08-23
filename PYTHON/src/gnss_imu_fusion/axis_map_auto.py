import numpy as np
import itertools


def _signed_perm_mats():
    perms = list(itertools.permutations([0, 1, 2], 3))
    signs = list(itertools.product([-1, 1], repeat=3))
    mats = []
    for p in perms:
        P = np.zeros((3, 3))
        P[0, p[0]] = 1
        P[1, p[1]] = 1
        P[2, p[2]] = 1
        for s in signs:
            S = np.diag(s)
            mats.append(S @ P)
    return mats


def choose_C_bs_from_static(mean_accel_sensor: np.ndarray):
    """Return sensor→body DCM so gravity maps to +Z (NED down).

    Parameters
    ----------
    mean_accel_sensor : array-like, shape (3,)
        Mean specific-force vector during a static window in the sensor frame.
    """
    gmag = float(np.linalg.norm(mean_accel_sensor))
    target = np.array([0.0, 0.0, gmag])  # +Z down in NED/body
    best = None
    best_err = 1e9
    for C in _signed_perm_mats():
        v = C @ mean_accel_sensor
        err = float(np.linalg.norm(v - target))
        if err < best_err:
            best_err = err
            best = C
    return best, best_err


def tilt_from_body_Z(g_body: np.ndarray) -> float:
    g = float(np.linalg.norm(g_body))
    if not np.isfinite(g) or g == 0:
        return float("nan")
    c = np.clip(g_body[2] / g, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))

