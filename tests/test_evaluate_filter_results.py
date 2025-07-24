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
    assert (tmp_path / "TEST_task7_residuals_position_velocity.pdf").exists()
    assert (tmp_path / "TEST_task7_attitude_angles_euler.pdf").exists()


def test_run_evaluation_npz_quat_longer(tmp_path):
    res_pos = np.zeros((5, 3))
    res_vel = np.ones((5, 3))
    t = np.linspace(0, 1, 5)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (6, 1))
    f = tmp_path / "data.npz"
    np.savez(f, residual_pos=res_pos, residual_vel=res_vel, time_residuals=t, attitude_q=quat)
    run_evaluation_npz(str(f), str(tmp_path), tag="TEST")
    assert (tmp_path / "TEST_task7_residuals_position_velocity.pdf").exists()
    assert (tmp_path / "TEST_task7_attitude_angles_euler.pdf").exists()


def test_run_evaluation_npz_diff_plot(tmp_path):
    t = np.linspace(0, 1, 11)
    res_pos = np.zeros((11, 3))
    res_vel = np.zeros((11, 3))
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (11, 1))
    fused_pos = np.column_stack([t, t, t])
    fused_vel = np.gradient(fused_pos, t, axis=0)
    f = tmp_path / "data.npz"
    np.savez(
        f,
        residual_pos=res_pos,
        residual_vel=res_vel,
        time_residuals=t,
        attitude_q=quat,
        time=t,
        pos_ned=fused_pos,
        vel_ned=fused_vel,
    )
    run_evaluation_npz(str(f), str(tmp_path), tag="TEST")
    assert (tmp_path / "TEST_task7_diff_truth_fused_over_time.pdf").exists()
