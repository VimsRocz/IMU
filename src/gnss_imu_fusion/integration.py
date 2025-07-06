from __future__ import annotations

import numpy as np

from ..utils import compute_C_ECEF_to_NED, gravity_ecef


def integrate_trajectory(
    acc_body: np.ndarray,
    imu_time: np.ndarray,
    C_B_N: np.ndarray,
    g_NED: np.ndarray | None = None,
    *,
    lat: np.ndarray | None = None,
    lon: np.ndarray | None = None,
    g_ecef: np.ndarray | None = None,
    h: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate body-frame accelerations to position and velocity in NED.

    If ``lat`` and ``lon`` are provided (or ``g_ecef`` is given), a
    position-dependent gravity vector is removed in the ECEF frame before
    converting the acceleration back to NED for integration.  Otherwise a
    constant ``g_NED`` is used for all epochs for backward compatibility.
    """

    n = len(imu_time)
    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    acc = np.zeros((n, 3))

    if lat is None or lon is None:
        if g_NED is None:
            raise ValueError("g_NED required when lat/lon not provided")
        for i in range(1, n):
            dt = imu_time[i] - imu_time[i - 1]
            f_ned = C_B_N @ acc_body[i]
            a_ned = f_ned + g_NED
            acc[i] = a_ned
            vel[i] = vel[i - 1] + a_ned * dt
            pos[i] = pos[i - 1] + vel[i] * dt
        return pos, vel, acc

    lat = np.asarray(lat)
    lon = np.asarray(lon)
    if h is None:
        h = np.zeros(n)
    else:
        h = np.asarray(h)

    if g_ecef is None:
        g_ecef = np.array([gravity_ecef(lat[i], lon[i], h[i]) for i in range(n)])

    for i in range(1, n):
        dt = imu_time[i] - imu_time[i - 1]
        C_e_n = compute_C_ECEF_to_NED(lat[i], lon[i])
        C_b_e = C_e_n.T @ C_B_N
        f_ecef = C_b_e @ acc_body[i]
        a_ecef = f_ecef - g_ecef[i]
        a_ned = C_e_n @ a_ecef
        acc[i] = a_ned
        vel[i] = vel[i - 1] + a_ned * dt
        pos[i] = pos[i - 1] + vel[i] * dt

    return pos, vel, acc
