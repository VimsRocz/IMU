from __future__ import annotations

import numpy as np


def to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(q_wxyz, float)
    if q.ndim == 1:
        return np.array([q[1], q[2], q[3], q[0]], float)
    return np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]]).astype(float)


def to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    q = np.asarray(q_xyzw, float)
    if q.ndim == 1:
        return np.array([q[3], q[0], q[1], q[2]], float)
    return np.column_stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]]).astype(float)


def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, float)
    if q.ndim == 1:
        n = float(np.linalg.norm(q))
        return q if n == 0 else (q / n)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def align_sign_to_ref(q: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Align quaternion signs sample-by-sample to match a reference series.

    Both q and ref are expected in the same convention (e.g., [w,x,y,z]).
    """
    q = np.asarray(q, float)
    ref = np.asarray(ref, float)
    if q.ndim == 1:
        s = 1.0 if float(np.dot(q, ref)) >= 0 else -1.0
        return s * q
    dots = np.sum(q * ref, axis=1)
    s = np.where(dots >= 0.0, 1.0, -1.0)[:, None]
    return q * s


def ensure_continuity(q: np.ndarray) -> np.ndarray:
    """Flip signs to enforce temporal continuity (q and -q are equivalent)."""
    q = normalize_quat(np.asarray(q, float))
    if q.ndim == 1:
        return q
    out = q.copy()
    for i in range(1, len(out)):
        if float(np.dot(out[i], out[i - 1])) < 0.0:
            out[i] *= -1.0
    return out


def _mean_geodesic_deg(q1_wxyz: np.ndarray, q2_wxyz: np.ndarray) -> float:
    q1 = normalize_quat(q1_wxyz)
    q2 = normalize_quat(q2_wxyz)
    d = np.sum(q1 * q2, axis=1)
    d = np.clip(np.abs(d), 0.0, 1.0)
    return float(np.degrees(2.0 * np.arccos(d)).mean())


def fix_estimator_quat_wxyz_against_truth(q_est: np.ndarray, q_truth_wxyz: np.ndarray) -> tuple[np.ndarray, dict]:
    """Return estimator quaternion interpreted as W,X,Y,Z to best match truth.

    Tries candidates:
      - as-is (assume W,X,Y,Z)
      - re-ordered from X,Y,Z,W -> W,X,Y,Z
    For each candidate also tries the conjugate (frame flip).
    Picks the combination with the lowest mean geodesic angle to the truth.

    Returns (q_est_wxyz_fixed, info_dict).
    """
    qT = ensure_continuity(normalize_quat(np.asarray(q_truth_wxyz, float)))
    qe_in = np.asarray(q_est, float)
    candidates: list[tuple[str, np.ndarray]] = []
    # as-is
    candidates.append(("wxyz", qe_in.copy()))
    # xyzw -> wxyz
    if qe_in.ndim == 2 and qe_in.shape[1] == 4:
        candidates.append(("xyzw→wxyz", to_wxyz(qe_in)))
    best = {"name": None, "conj": False, "err_deg": float("inf")}
    best_q = qe_in
    for name, qe in candidates:
        qe = ensure_continuity(normalize_quat(qe))
        err_as_is = _mean_geodesic_deg(qe, qT)
        qe_conj = qe * np.array([1.0, -1.0, -1.0, -1.0])
        err_conj = _mean_geodesic_deg(qe_conj, qT)
        if err_conj < err_as_is:
            err, qfix, conj = err_conj, qe_conj, True
        else:
            err, qfix, conj = err_as_is, qe, False
        if err < best["err_deg"]:
            best = {"name": name, "conj": conj, "err_deg": err}
            best_q = qfix
    return best_q, best
