import numpy as np
from src.ins.mechanisation import mechanise_static


def test_static_velocity_drift():
    dt = 0.01
    N = 200
    from src.earth_model import gravity_ned
    g = gravity_ned(0.0, 0.0)[2]
    acc_body = np.tile(np.array([0.0, 0.0, g]), (N, 1))
    gyro_body = np.zeros((N, 3))
    v_n = mechanise_static(acc_body, gyro_body, dt)
    assert abs(v_n[-1, 2]) < 0.05
