from __future__ import annotations

import numpy as np


def integrate_trajectory(acc_body: np.ndarray, imu_time: np.ndarray, C_B_N: np.ndarray, g_NED: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate body-frame accelerations to position and velocity in NED."""
    n = len(imu_time)
    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    acc = np.zeros((n, 3))
    for i in range(1, n):
        dt = imu_time[i] - imu_time[i - 1]
        f_ned = C_B_N @ acc_body[i]
        a_ned = f_ned + g_NED
        acc[i] = a_ned
        vel[i] = vel[i - 1] + a_ned * dt
        pos[i] = pos[i - 1] + vel[i] * dt
    return pos, vel, acc
