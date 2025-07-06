import pytest

from src.kalman import GNSSIMUKalman
from src.constants import GRAVITY

np = pytest.importorskip("numpy")


def test_predict_handles_gravity():
    kf = GNSSIMUKalman()
    kf.init_state(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    dt = 1.0
    R_bn = np.eye(3)
    acc_meas = np.array([0.0, 0.0, -GRAVITY])
    g_n = np.array([0.0, 0.0, GRAVITY])

    kf.predict(dt, R_bn, acc_meas, g_n)

    assert np.allclose(kf.kf.x[3:6], 0.0, atol=1e-6)
    assert np.allclose(kf.kf.x[0:3], 0.0, atol=1e-6)
