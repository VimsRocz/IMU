import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("filterpy")

from filterpy.kalman import KalmanFilter, rts_smoother as fp_rts

from src.kalman import rts_smoother


def test_rts_smoother_constant_velocity():
    rng = np.random.default_rng(0)
    dt = 1.0
    F = np.array([[1.0, dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    q = 0.1
    Q = np.array(
        [
            [0.25 * dt ** 4, 0.5 * dt ** 3],
            [0.5 * dt ** 3, dt ** 2],
        ]
    ) * q ** 2
    R = np.array([[0.1 ** 2]])

    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.F = F
    kf.H = H
    kf.Q = Q
    kf.R = R
    kf.x = np.array([0.0, 1.0])
    kf.P = np.eye(2) * 0.1

    N = 5
    xs = np.zeros((N, 2))
    Ps = np.zeros((N, 2, 2))
    Fs = [F for _ in range(N)]
    Qs = [Q for _ in range(N)]

    x_true = np.array([0.0, 1.0])
    for k in range(N):
        z = H @ x_true + rng.normal(scale=0.1)
        kf.predict()
        kf.update(z)
        xs[k] = kf.x
        Ps[k] = kf.P
        x_true = F @ x_true

    xs_sm, Ps_sm = rts_smoother(xs, Ps, Fs, Qs)

    xs_expected, Ps_expected, _, _ = fp_rts(xs, Ps, Fs, Qs)

    assert np.allclose(xs_sm, xs_expected, atol=1e-8)
    assert np.allclose(Ps_sm, Ps_expected, atol=1e-8)

