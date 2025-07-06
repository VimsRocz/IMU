import numpy as np
from filterpy.kalman import KalmanFilter

class GNSSIMUKalman:
    """Simple GNSS/IMU Kalman filter with bias states.

    Parameters
    ----------
    gnss_weight : float
        Scaling applied to the GNSS measurement covariance.
    accel_noise : float
        Process noise standard deviation for acceleration.
    accel_bias_noise : float
        Random-walk noise for accelerometer bias states.
    gyro_bias_noise : float
        Random-walk noise for gyroscope bias states.
    pos_proc_noise : float
        Process noise standard deviation applied to the position states.
    vel_proc_noise : float
        Additional process noise for the velocity states.
    pos_meas_noise : float
        Measurement noise standard deviation for position updates.
    vel_meas_noise : float
        Measurement noise standard deviation for velocity updates.
    """

    def __init__(
        self,
        gnss_weight: float = 1.0,
        accel_noise: float = 0.1,
        accel_bias_noise: float = 1e-5,
        gyro_bias_noise: float = 1e-5,
        pos_proc_noise: float = 0.0,
        vel_proc_noise: float = 0.0,
        pos_meas_noise: float = 1.0,
        vel_meas_noise: float = 1.0,
    ) -> None:
        self.gnss_weight = gnss_weight
        self.accel_noise = accel_noise
        self.accel_bias_noise = accel_bias_noise
        self.gyro_bias_noise = gyro_bias_noise
        self.pos_proc_noise = pos_proc_noise
        self.vel_proc_noise = vel_proc_noise
        self.pos_meas_noise = pos_meas_noise
        self.vel_meas_noise = vel_meas_noise
        self.kf = KalmanFilter(dim_x=12, dim_z=6)

    def init_state(self, position, velocity, acc_bias, gyro_bias):
        self.kf.x = np.hstack([position, velocity, acc_bias, gyro_bias])
        self.kf.P = np.eye(12) * 1.0
        self.kf.H = np.zeros((6, 12))
        self.kf.H[0:3, 0:3] = np.eye(3)
        self.kf.H[3:6, 3:6] = np.eye(3)
        R = np.zeros((6, 6))
        R[0:3, 0:3] = np.eye(3) * (self.pos_meas_noise ** 2) / self.gnss_weight
        R[3:6, 3:6] = np.eye(3) * (self.vel_meas_noise ** 2) / self.gnss_weight
        self.kf.R = R

    def predict(self, dt, R_bn, acc_meas, g_n):
        F = np.eye(12)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R_bn * dt
        Q = np.zeros((12, 12))
        q_pos = (self.pos_proc_noise ** 2) * dt
        if q_pos:
            Q[0:3, 0:3] = np.eye(3) * q_pos
        q_acc = (self.accel_noise ** 2) * dt
        Q[3:6, 3:6] = np.eye(3) * q_acc
        q_vel = (self.vel_proc_noise ** 2) * dt
        if q_vel:
            Q[3:6, 3:6] += np.eye(3) * q_vel
        q_abias = (self.accel_bias_noise ** 2) * dt
        q_gbias = (self.gyro_bias_noise ** 2) * dt
        Q[6:9, 6:9] = np.eye(3) * q_abias
        Q[9:12, 9:12] = np.eye(3) * q_gbias
        self.kf.F = F
        self.kf.Q = Q
        self.kf.predict()
        # manual integration of position/velocity
        acc_body = acc_meas - self.kf.x[6:9]
        acc_n = R_bn @ acc_body

        # Compensate gravity once in navigation frame
        acc_n -= g_n
        self.kf.x[3:6] += acc_n * dt
        self.kf.x[0:3] += self.kf.x[3:6] * dt

    def update_gnss(self, pos_meas, vel_meas):
        """Update state with GNSS measurement and return residual."""
        z = np.hstack([pos_meas, vel_meas])
        R = np.zeros((6, 6))
        R[0:3, 0:3] = np.eye(3) * (self.pos_meas_noise ** 2) / self.gnss_weight
        R[3:6, 3:6] = np.eye(3) * (self.vel_meas_noise ** 2) / self.gnss_weight
        self.kf.R = R
        pred = self.kf.H @ self.kf.x
        residual = z - pred
        S = self.kf.H @ self.kf.P @ self.kf.H.T + self.kf.R
        self.kf.update(z)
        return self.kf.x.copy(), residual, S

def rts_smoother(X, P, F_list, Q_list):
    """Rauch-Tung-Striebel smoother."""
    N = len(X)
    x_smooth = X.copy()
    P_smooth = P.copy()
    for k in range(N - 2, -1, -1):
        F = F_list[k]
        C = P[k] @ F.T @ np.linalg.inv(F @ P[k] @ F.T + Q_list[k])
        x_smooth[k] = X[k] + C @ (x_smooth[k+1] - F @ X[k])
        P_smooth[k] = P[k] + C @ (P_smooth[k+1] - F @ P[k] @ F.T - Q_list[k]) @ C.T
    return x_smooth, P_smooth
