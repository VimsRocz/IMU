import numpy as np
from kalman_filter import init_bias_kalman, inject_zupt


def test_bias_states_and_zupt():
    kf = init_bias_kalman(0.1, np.eye(3), np.eye(3))
    kf.x[:] = 0.0
    kf.x[3:6, 0] = np.array([1.0, -2.0, 0.5])
    inject_zupt(kf)
    assert np.allclose(kf.x[3:6, 0], 0.0, atol=1e-3)
    assert np.allclose(np.diag(kf.Q)[10:13], 1e-8)
