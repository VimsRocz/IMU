import pytest

np = pytest.importorskip("numpy")
from src.evaluate_filter_results import run_evaluation_npz


def test_run_evaluation_npz_mismatched_lengths(tmp_path):
    res_pos = np.zeros((5, 3))
    res_vel = np.ones((4, 3))
    t = np.linspace(0, 1, 6)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (4, 1))
    f = tmp_path / "data.npz"
    np.savez(f, residual_pos=res_pos, residual_vel=res_vel, time_residuals=t, attitude_q=quat)
    run_evaluation_npz(str(f), str(tmp_path))
    assert (tmp_path / "residuals_position_velocity.pdf").exists()
    assert (tmp_path / "attitude_angles_euler.pdf").exists()
