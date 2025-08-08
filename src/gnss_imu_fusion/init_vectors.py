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


def triad_basis(vec1: np.ndarray, vec2: np.ndarray) -> np.ndarray:
    """Return an orthonormal basis using the classic TRIAD construction.

    Parameters
    ----------
    vec1 : np.ndarray
        Primary reference vector.
    vec2 : np.ndarray
        Secondary reference vector.

    Returns
    -------
    np.ndarray
        3×3 matrix with ``vec1`` as the first column.

    Notes
    -----
    Mirrors the helper used throughout the MATLAB codebase. ``vec1`` forms the
    primary axis of the basis. ``vec2`` is used to compute the second axis. If
    the cross product of the two vectors is close to zero (i.e. they are
    collinear) an arbitrary orthogonal axis is chosen to ensure a valid basis.
    """

    v1 = np.asarray(vec1, dtype=float).reshape(3)
    v2 = np.asarray(vec2, dtype=float).reshape(3)

    t1 = v1 / np.linalg.norm(v1)
    t2_temp = np.cross(t1, v2)

    if np.linalg.norm(t2_temp) < 1e-10:
        # Fallback axis roughly orthogonal to t1
        tmp = (
            np.array([1.0, 0.0, 0.0])
            if abs(t1[0]) < abs(t1[1])
            else np.array([0.0, 1.0, 0.0])
        )
        t2_temp = np.cross(t1, tmp)

    t2 = t2_temp / np.linalg.norm(t2_temp)
    t3 = np.cross(t1, t2)

    return np.column_stack((t1, t2, t3))


def triad_svd(
    body_vec1: np.ndarray,
    body_vec2: np.ndarray,
    ref_vec1: np.ndarray,
    ref_vec2: np.ndarray,
    w1: float = 0.9999,
    w2: float = 0.0001,
) -> np.ndarray:
    """Return body->NED rotation from two vector pairs using SVD."""
    return svd_alignment(
        [body_vec1, body_vec2],
        [ref_vec1, ref_vec2],
        [w1, w2],
    )


def butter_lowpass_filter(
    data: np.ndarray, cutoff: float = 5.0, fs: float = 400.0, order: int = 4
) -> np.ndarray:
    """Apply a zero-phase Butterworth low-pass filter to the data."""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    return filtfilt(b, a, data, axis=0)


def basic_butterworth_filter(
    data: np.ndarray, cutoff: float = 5.0, fs: float = 400.0, order: int = 4
) -> np.ndarray:
    """Butterworth low-pass filter implemented without design helper functions.

    Parameters
    ----------
    data : np.ndarray
        ``N×M`` array of samples (time along axis 0).
    cutoff : float, optional
        Cut-off frequency in Hz, by default 5.0.
    fs : float, optional
        Sampling frequency in Hz, by default 400.0.
    order : int, optional
        Filter order, by default 4.

    Returns
    -------
    np.ndarray
        Filtered array of the same shape as ``data``.

    Notes
    -----
    This mirrors the MATLAB implementation used in the companion code base and
    is provided for situations where the standard :func:`butter` helper from
    :mod:`scipy` is unavailable. ``scipy.signal.lfilter`` is still used for the
    actual filtering steps.
    """

    from scipy.signal import lfilter

    data = np.asarray(data, dtype=float)
    if data.ndim == 1:
        data = data[:, None]

    # Pre-warp frequency for bilinear transform
    warped = 2 * fs * np.tan(np.pi * cutoff / fs)
    k = np.arange(order)
    angles = np.pi / 2 + (2 * k + 1) * np.pi / (2 * order)
    poles_analog = warped * np.exp(1j * angles)
    poles_digital = (2 * fs + poles_analog) / (2 * fs - poles_analog)
    zeros_digital = -np.ones(order)
    gain_analog = warped**order
    gain_digital = np.real(gain_analog / np.prod(2 * fs - poles_analog))

    b = np.real(np.poly(zeros_digital)) * gain_digital
    a = np.real(np.poly(poles_digital))

    y = lfilter(b, a, data, axis=0)
    filt = lfilter(b, a, y[::-1], axis=0)[::-1]

    return filt.squeeze()


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
    """Return gravity and Earth-rate angle errors for a body->NED DCM.

    Parameters
    ----------
    C_bn : np.ndarray
        Direction cosine matrix rotating vectors from the body frame to NED.
    g_body : np.ndarray
        Gravity vector measured in the body frame.
    omega_ie_body : np.ndarray
        Earth rotation vector measured in the body frame.
    g_ref_ned : np.ndarray
        Reference gravity vector in NED coordinates.
    omega_ref_ned : np.ndarray
        Reference Earth rotation vector in NED coordinates.

    Notes
    -----
    Both ``omega_pred_ned`` and ``omega_ref_ned`` are expressed in the NED frame
    so that the returned ``earth_err`` directly compares the two.  The function
    mirrors the MATLAB version to maintain cross-language parity.
    """
    g_pred_ned = C_bn @ g_body
    omega_pred_ned = C_bn @ omega_ie_body
    grav_err = angle_between(g_pred_ned, g_ref_ned)
    earth_err = angle_between(omega_pred_ned, omega_ref_ned)
    return grav_err, earth_err


def davenport_q_method(
    v1_b: np.ndarray,
    v2_b: np.ndarray,
    v1_n: np.ndarray,
    v2_n: np.ndarray,
    w1: float = 0.9999,
    w2: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return rotation matrix and quaternion using Davenport's Q-method."""

    if w2 is None:
        w2 = 1.0 - w1

    v1_b = np.asarray(v1_b)
    v2_b = np.asarray(v2_b)
    v1_n = np.asarray(v1_n)
    v2_n = np.asarray(v2_n)

    B = w1 * np.outer(
        v1_n / np.linalg.norm(v1_n), v1_b / np.linalg.norm(v1_b)
    ) + w2 * np.outer(v2_n / np.linalg.norm(v2_n), v2_b / np.linalg.norm(v2_b))
    S = B + B.T
    sigma = np.trace(B)
    Z = np.array([B[1, 2] - B[2, 1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]])
    K = np.zeros((4, 4))
    K[0, 0] = sigma
    K[0, 1:] = Z
    K[1:, 0] = Z
    K[1:, 1:] = S - sigma * np.eye(3)

    eigvals, eigvecs = np.linalg.eigh(K)
    q_opt = eigvecs[:, np.argmax(eigvals)]
    if q_opt[0] < 0:
        q_opt = -q_opt

    q = np.array([q_opt[0], -q_opt[1], -q_opt[2], -q_opt[3]])

    qw, qx, qy, qz = q
    R = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ]
    )

    return R, q


def svd_wahba(
    n: np.ndarray,
    b: np.ndarray,
    w: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Return body->NED rotation solving Wahba's problem via SVD.

    Parameters
    ----------
    n : np.ndarray
        3xK matrix of reference vectors expressed in the NED frame.
    b : np.ndarray
        3xK matrix of the same vectors measured in the body frame.
    w : Optional[np.ndarray]
        Optional 1xK array of weights. Defaults to equal weighting.
    """

    n = np.asarray(n)
    b = np.asarray(b)
    if n.shape != b.shape or n.shape[0] != 3:
        raise ValueError("n and b must be 3xK matrices")

    if w is None:
        w = np.ones(n.shape[1])
    else:
        w = np.asarray(w)
        if w.shape != (n.shape[1],):
            raise ValueError("w must have length K")

    H = np.zeros((3, 3))
    for k in range(n.shape[1]):
        H += w[k] * np.outer(b[:, k], n[:, k])
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    return U @ D @ Vt
