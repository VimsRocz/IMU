import numpy as np

from src.kalman import GNSSIMUKalman


def rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rms_error(a, b):
    return np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1)))


def test_lever_arm_compensation_reduces_error():
    dt = 0.1
    steps = 100
    omega = np.array([0.0, 0.0, 1.0])
    lever = np.array([1.0, 0.0, 0.0])

    # initial orientation and state
    R0 = rotz(0.0)
    v0 = R0 @ np.cross(omega, lever)
    p0 = R0 @ lever

    kf_no = GNSSIMUKalman()
    kf_no.init_state(p0, v0, np.zeros(3), np.zeros(3), R_bn0=R0, omega_b0=omega)

    kf_la = GNSSIMUKalman(lever_arm=lever)
    kf_la.init_state(p0, v0, np.zeros(3), np.zeros(3), R_bn0=R0, omega_b0=omega)

    pos_no = []
    pos_la = []
    truth = []
    for i in range(1, steps + 1):
        theta = omega[2] * dt * i
        R = rotz(theta)
        kf_no.predict(dt, R, np.zeros(3), np.zeros(3), omega)
        kf_la.predict(dt, R, np.zeros(3), np.zeros(3), omega)
        pos_no.append(kf_no.kf.x[:3].copy())
        pos_la.append(kf_la.kf.x[:3].copy())
        truth.append(R @ lever)

    pos_no = np.array(pos_no)
    pos_la = np.array(pos_la)
    truth = np.array(truth)

    err_no = rms_error(pos_no, truth)
    err_la = rms_error(pos_la, truth)

    assert err_la < err_no
