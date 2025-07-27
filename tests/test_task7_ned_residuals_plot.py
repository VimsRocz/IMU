import numpy as np
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
from src.task7_ned_residuals_plot import compute_residuals, plot_residuals


def test_plot_residuals(tmp_path: Path):
    t = np.linspace(0, 1, 5)
    pos_est = np.vstack([t, t, t]).T
    vel_est = np.gradient(pos_est, t, axis=0)
    pos_truth = pos_est + 0.1
    vel_truth = vel_est + 0.1
    res_pos, res_vel, res_acc = compute_residuals(
        t, pos_est, vel_est, pos_truth, vel_truth
    )
    out_dir = tmp_path / "TEST"
    plot_residuals(t, res_pos, res_vel, res_acc, "TEST", out_dir)
    assert (out_dir / "TEST_task7_ned_residuals.pdf").exists()
    assert (out_dir / "TEST_task7_ned_residual_norms.pdf").exists()
