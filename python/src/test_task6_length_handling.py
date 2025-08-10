import pytest

np = pytest.importorskip("numpy")
import warnings

from task6_overlay_plot import interpolate_truth, ensure_equal_length


def test_truncate_to_shorter_length():
    t_est_full = np.linspace(0, 4, 5)
    pos_est = np.vstack([t_est_full, t_est_full, t_est_full]).T
    vel_est = np.zeros_like(pos_est)
    acc_est = np.zeros_like(pos_est)

    t_truth = np.linspace(0, 2, 3)
    pos_truth = np.vstack([t_truth, t_truth, t_truth]).T
    vel_truth = np.zeros_like(pos_truth)
    acc_truth = np.zeros_like(pos_truth)

    # simulate aligning to shorter truth time vector
    t_est = t_est_full[: len(t_truth)]
    pos_truth_i = interpolate_truth(t_est, t_truth, pos_truth)
    vel_truth_i = interpolate_truth(t_est, t_truth, vel_truth)
    acc_truth_i = interpolate_truth(t_est, t_truth, acc_truth)

    with warnings.catch_warnings(record=True) as w:
        (
            t_out,
            pos_est_out,
            vel_est_out,
            acc_est_out,
            pos_truth_out,
            vel_truth_out,
            acc_truth_out,
        ) = ensure_equal_length(
            t_est,
            pos_est,
            vel_est,
            acc_est,
            pos_truth_i,
            vel_truth_i,
            acc_truth_i,
        )
        assert len(w) == 1

    assert len(pos_est_out) == len(pos_truth_out) == len(t_out)
    assert len(t_out) == len(t_truth)
