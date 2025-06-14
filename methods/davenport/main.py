import numpy as np
from imu_fusion.data import load_gnss_csv, load_imu_dat
from imu_fusion.wahba import davenport_q_method
from imu_fusion.attitude import rot_to_quaternion


def main() -> None:
    gnss = load_gnss_csv('GNSS_X001.csv')
    imu = load_imu_dat('IMU_X001.dat')

    lat = np.deg2rad(float(gnss.iloc[0].get('Latitude_deg', 0.0)))
    g_ned = np.array([0.0, 0.0, 9.81])
    omega_ie = 7.2921159e-5 * np.array([np.cos(lat), 0.0, -np.sin(lat)])

    dt = 1.0 / 400.0
    acc = imu[:, 5:8] / dt
    gyro = imu[:, 2:5] / dt
    N = min(4000, len(acc))
    g_body = -np.mean(acc[:N], axis=0)
    omega_body = np.mean(gyro[:N], axis=0)

    R = davenport_q_method(g_body, omega_body, g_ned, omega_ie)
    q = rot_to_quaternion(R)
    print('Davenport quaternion (body to NED):', q)


if __name__ == '__main__':
    main()
