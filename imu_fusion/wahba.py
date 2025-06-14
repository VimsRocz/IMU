import numpy as np


def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("zero vector")
    return v / n


def triad_method(v1_b: np.ndarray, v2_b: np.ndarray,
                 v1_n: np.ndarray, v2_n: np.ndarray) -> np.ndarray:
    """Return body-to-NED rotation matrix using the TRIAD algorithm."""
    t1_b = _normalize(v1_b)
    t2_b = _normalize(np.cross(t1_b, v2_b))
    t3_b = np.cross(t1_b, t2_b)

    t1_n = _normalize(v1_n)
    t2_n = _normalize(np.cross(t1_n, v2_n))
    t3_n = np.cross(t1_n, t2_n)

    R = np.column_stack((t1_n, t2_n, t3_n)) @ np.column_stack((t1_b, t2_b, t3_b)).T
    return R


def davenport_q_method(v1_b: np.ndarray, v2_b: np.ndarray,
                        v1_n: np.ndarray, v2_n: np.ndarray,
                        w1: float = 1.0, w2: float = 1.0) -> np.ndarray:
    """Return body-to-NED rotation matrix using Davenport's Q-Method."""
    B = w1 * np.outer(_normalize(v1_n), _normalize(v1_b)) + \
        w2 * np.outer(_normalize(v2_n), _normalize(v2_b))
    sigma = np.trace(B)
    S = B + B.T
    Z = np.array([B[1,2] - B[2,1], B[2,0] - B[0,2], B[0,1] - B[1,0]])
    K = np.zeros((4,4))
    K[0,0] = sigma
    K[0,1:] = Z
    K[1:,0] = Z
    K[1:,1:] = S - sigma * np.eye(3)
    eigvals, eigvecs = np.linalg.eigh(K)
    q = eigvecs[:, np.argmax(eigvals)]
    if q[0] < 0:
        q = -q
    q = np.array([q[0], -q[1], -q[2], -q[3]])  # conjugate to get body->NED
    return quaternion_to_rot(q)


def svd_method(v1_b: np.ndarray, v2_b: np.ndarray,
                v1_n: np.ndarray, v2_n: np.ndarray,
                w1: float = 1.0, w2: float = 1.0) -> np.ndarray:
    """Return body-to-NED rotation matrix using the SVD algorithm."""
    M = w1 * np.outer(_normalize(v1_n), _normalize(v1_b)) + \
        w2 * np.outer(_normalize(v2_n), _normalize(v2_b))
    U, _, Vt = np.linalg.svd(M)
    R = U @ np.diag([1, 1, np.linalg.det(U) * np.linalg.det(Vt)]) @ Vt
    return R


def quaternion_to_rot(q: np.ndarray) -> np.ndarray:
    """Convert [qw, qx, qy, qz] quaternion to rotation matrix."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])

__all__ = [
    "triad_method",
    "davenport_q_method",
    "svd_method",
]
