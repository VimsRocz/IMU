"""Vector initialisation utilities extracted from the original script."""

from typing import Iterable, Tuple, Optional

import numpy as np
from scipy.signal import butter, filtfilt


def average_rotation_matrices(rotations: Iterable[np.ndarray]) -> np.ndarray:
    """Average a list of rotation matrices and re-orthonormalise."""
    A = sum(rotations) / len(rotations)
    U, _, Vt = np.linalg.svd(A)
    return U @ Vt


def svd_alignment(
    body_vecs: Iterable[np.ndarray],
    ref_vecs: Iterable[np.ndarray],
    weights: Optional[Iterable[float]] = None,
) -> np.ndarray:
    """Return body->NED rotation using SVD for an arbitrary number of vector pairs."""
    if weights is None:
        weights = np.ones(len(list(body_vecs)))
    B = sum(
        w * np.outer(r / np.linalg.norm(r), b / np.linalg.norm(b))
        for b, r, w in zip(body_vecs, ref_vecs, weights)
    )
    U, _, Vt = np.linalg.svd(B)
    M = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ M @ Vt


def butter_lowpass_filter(data: np.ndarray, cutoff: float = 5.0, fs: float = 400.0, order: int = 4) -> np.ndarray:
    """Apply a zero-phase Butterworth low-pass filter to the data."""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    return filtfilt(b, a, data, axis=0)


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Return the angle in degrees between two vectors."""
    a = np.asarray(a)
    b = np.asarray(b)
    if a.shape != (3,) or b.shape != (3,):
        raise ValueError("angle_between expects 3D vectors")
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na == 0 or nb == 0:
        return float("nan")
    cosang = np.dot(a, b) / (na * nb)
    cosang = np.clip(cosang, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosang)))


def compute_wahba_errors(
    C_bn: np.ndarray,
    g_body: np.ndarray,
    omega_ie_body: np.ndarray,
    g_ref_ned: np.ndarray,
    omega_ref_ned: np.ndarray,
) -> Tuple[float, float]:
    """Return gravity and Earth-rate angle errors for a DCM."""
    g_pred_ned = C_bn @ g_body
    omega_pred_ned = C_bn @ omega_ie_body
    grav_err = angle_between(g_pred_ned, g_ref_ned)
    earth_err = angle_between(omega_pred_ned, omega_ref_ned)
    return grav_err, earth_err

