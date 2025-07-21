from __future__ import annotations

import numpy as np
from typing import Optional, Tuple

from ..utils import compute_C_ECEF_to_NED, gravity_ecef


def integrate_trajectory(
    acc_body: np.ndarray,
    imu_time: np.ndarray,
    C_B_N: np.ndarray,
    g_NED: Optional[np.ndarray] = None,
    *,
    lat: Optional[np.ndarray] = None,
    lon: Optional[np.ndarray] = None,
    g_ecef: Optional[np.ndarray] = None,
    h: Optional[np.ndarray] = None,
    ref_lat: Optional[float] = None,
    ref_lon: Optional[float] = None,
    ref_ecef: Optional[np.ndarray] = None,
    debug: bool = False,
) -> Tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    Optional[np.ndarray],
    Optional[np.ndarray],
]:
    """Integrate body-frame accelerations to position and velocity.

    If ``lat`` and ``lon`` are provided (or ``g_ecef`` is given), a
    position-dependent gravity vector is removed in the ECEF frame before
    converting the acceleration back to NED for integration.  Otherwise a
    constant ``g_NED`` is used for all epochs for backward compatibility.

    When ``debug`` is ``True`` the function prints intermediate velocity
    values at the start, middle and end of the integration to help diagnose
    divergence issues.
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
            a_ned = f_ned - g_NED
            acc[i] = a_ned
            vel[i] = vel[i - 1] + a_ned * dt
            pos[i] = pos[i - 1] + vel[i] * dt
            if debug and i in {1, n // 2, n - 1}:
                print(
                    f"[DEBUG] i={i} dt={dt:.4f} vel={vel[i]} acc={a_ned}",
                )
        return pos, vel, acc, None, None

    lat = np.asarray(lat)
    lon = np.asarray(lon)
    if h is None:
        h = np.zeros(n)
    else:
        h = np.asarray(h)

    if g_ecef is None:
        g_ecef = np.array([gravity_ecef(lat[i], lon[i], h[i]) for i in range(n)])

    pos_ecef = np.zeros((n, 3))
    vel_ecef = np.zeros((n, 3))
    acc_ecef = np.zeros((n, 3))

    for i in range(1, n):
        dt = imu_time[i] - imu_time[i - 1]
        C_e_n = compute_C_ECEF_to_NED(lat[i], lon[i])
        C_b_e = C_e_n.T @ C_B_N
        f_ecef = C_b_e @ acc_body[i]
        a_ecef = f_ecef - g_ecef[i]
        acc_ecef[i] = a_ecef
        vel_ecef[i] = vel_ecef[i - 1] + a_ecef * dt
        pos_ecef[i] = pos_ecef[i - 1] + vel_ecef[i] * dt
        if debug and i in {1, n // 2, n - 1}:
            print(
                f"[DEBUG] i={i} dt={dt:.4f} vel_ecef={vel_ecef[i]} acc_ecef={a_ecef}"
            )

    if ref_lat is None:
        ref_lat = float(lat[0])
    if ref_lon is None:
        ref_lon = float(lon[0])
    if ref_ecef is None:
        ref_ecef = np.zeros(3)
    C_ref = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    pos = np.array([C_ref @ (p - ref_ecef) for p in pos_ecef])
    vel = np.array([C_ref @ v for v in vel_ecef])
    acc = np.array([C_ref @ a for a in acc_ecef])
    if debug:
        print(f"First accel_ned: {acc[1]}, Last accel_ned: {acc[-1]}")
        print(f"First velocity_ned: {vel[1]}, Last velocity_ned: {vel[-1]}")

    return pos, vel, acc, pos_ecef, vel_ecef
