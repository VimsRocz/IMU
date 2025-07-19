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
    run_evaluation_npz(str(f), str(tmp_path), tag="TEST")
    assert (tmp_path / "TEST_residuals_position_velocity.pdf").exists()
    assert (tmp_path / "TEST_attitude_angles_euler.pdf").exists()


def test_run_evaluation_npz_long_quaternion(tmp_path):
    """Quaternion array longer than residual arrays should be truncated."""
    res_pos = np.zeros((5, 3))
    res_vel = np.ones((5, 3))
    t = np.linspace(0, 1, 5)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (7, 1))
    f = tmp_path / "data_long_q.npz"
    np.savez(f, residual_pos=res_pos, residual_vel=res_vel, time_residuals=t, attitude_q=quat)
    run_evaluation_npz(str(f), str(tmp_path), tag="TESTQ")
    assert (tmp_path / "TESTQ_residuals_position_velocity.pdf").exists()
    assert (tmp_path / "TESTQ_attitude_angles_euler.pdf").exists()
