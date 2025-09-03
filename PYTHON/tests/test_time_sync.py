import numpy as np
from PYTHON.scripts.eval.run_evaluation_npz import (
    _trim_to_overlap,
    estimate_time_offset,
    run_evaluation_npz,
)


def test_trim_to_overlap():
    t_truth = np.linspace(0, 10, 11)
    x_truth = np.vstack([t_truth, t_truth, t_truth]).T
    t_est = np.linspace(2, 8, 7)
    x_est = np.vstack([t_est, t_est, t_est]).T
    t, xt, xe = _trim_to_overlap(t_truth, x_truth, t_est, x_est)
    assert t[0] == 2 and t[-1] == 8
    assert xt.shape == xe.shape == (7, 3)


def test_estimate_time_offset():
    fs = 10.0
    t = np.arange(0, 1, 1 / fs)
    sig = np.sin(2 * np.pi * t)
    shifted = np.sin(2 * np.pi * (t + 0.2))
    offset = estimate_time_offset(sig, shifted, fs)
    assert abs(abs(offset) - 0.2) < 0.02


def test_run_evaluation_respects_shift(tmp_path):
    t_est = np.linspace(0, 5, 6)
    vel_est = np.vstack([t_est, t_est, t_est]).T
    t_truth = t_est + 0.5
    vel_truth = vel_est.copy()
    f = tmp_path / "data.npz"
    np.savez(f, time=t_est, vel_ned=vel_est, truth_time=t_truth, truth_vel_ned=vel_truth, t_shift=0.5)
    out = run_evaluation_npz(str(f))
    assert np.allclose(out["rmse_vel"], 0.0)
