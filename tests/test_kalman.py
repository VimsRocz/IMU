import numpy as np
from imu_fusion.kalman import kalman_with_residuals, kalman_with_bias


def test_kalman_with_residuals_constant_velocity():
    zs = np.arange(1, 6).reshape(-1, 1).astype(float)
    F = np.array([[1.0, 1.0], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q = np.zeros((2, 2))
    R = np.eye(1) * 0.1
    x0 = np.array([0.0, 0.0])

    xs, res = kalman_with_residuals(zs, F, H, Q, R, x0)

    expected_res = np.array([
        1.0,
        0.57142857,
        0.19298246,
        0.08300908,
        0.04485263,
    ]).reshape(-1, 1)
    expected_xs = np.array(
        [
            [0.95238095, 0.47619048],
            [1.92982456, 0.87719298],
            [2.95719844, 0.95979248],
            [3.97266126, 0.98248612],
            [4.98125335, 0.99089448],
        ]
    )

    assert np.allclose(xs, expected_xs)
    assert np.allclose(res, expected_res)


def test_kalman_with_bias_initial_bias():
    zs = np.zeros((3, 1))
    F = np.eye(1)
    H = np.eye(1)
    Q = np.zeros((1, 1))
    R = np.eye(1)
    x0 = np.array([0.0])
    xs, _ = kalman_with_bias(zs, F, H, Q, R, x0, initial_bias=np.array([0.5, 0.5, 0.5]))
    assert xs.shape[1] == 4
    assert np.allclose(xs[0, -3:], [0.5, 0.5, 0.5])
