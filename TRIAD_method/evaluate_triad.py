import numpy as np
import pandas as pd
from imu_fusion.data import load_gnss_csv, load_imu_dat
from imu_fusion.wahba import triad_method
from imu_fusion.attitude import (
    rot_to_quaternion,
    estimate_initial_orientation,
    quat_multiply,
    quat_normalize,
)


def estimate_orientation(gnss_file: str, imu_file: str, n_static: int = 4000) -> tuple[np.ndarray, float]:
    """Return TRIAD quaternion and angle error to GNSS yaw estimate."""
    gnss = load_gnss_csv(gnss_file)
    imu = load_imu_dat(imu_file)

    lat = np.deg2rad(float(gnss.iloc[0].get("Latitude_deg", 0.0)))
    g_ned = np.array([0.0, 0.0, 9.81])
    omega_ie = 7.2921159e-5 * np.array([np.cos(lat), 0.0, -np.sin(lat)])

    dt = 1.0 / 400.0
    acc = imu[:, 5:8] / dt
    gyro = imu[:, 2:5] / dt
    N = min(n_static, len(acc))
    g_body = -np.mean(acc[:N], axis=0)
    omega_body = np.mean(gyro[:N], axis=0)

    R = triad_method(g_body, omega_body, g_ned, omega_ie)
    q_triad = rot_to_quaternion(R)

    q_ref = estimate_initial_orientation(imu, gnss)
    q_err = quat_multiply(q_triad, np.array([q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]]))
    ang_err = 2 * np.arccos(np.clip(quat_normalize(q_err)[0], -1.0, 1.0))
    return q_triad, np.degrees(ang_err)


def main() -> None:
    gnss_files = ["GNSS_X001.csv", "GNSS_X002.csv"]
    imu_files = ["IMU_X001.dat", "IMU_X002.dat", "IMU_X003.dat"]

    for gnss in gnss_files:
        for imu in imu_files:
            q, err = estimate_orientation(gnss, imu)
            print(f"Orientation from {gnss} + {imu}: {q}, angle error to GNSS yaw {err:.2f} deg")


if __name__ == "__main__":
    main()
