import numpy as np
from src.plot_overlay import plot_overlay


def test_plot_overlay_with_truth(tmp_path):
    t = np.linspace(0, 1, 5)
    pos = np.vstack([t, t, t]).T
    vel = pos * 0
    acc = pos * 0
    plot_overlay(
        "ECEF",
        "TEST",
        t,
        pos,
        vel,
        acc,
        t,
        pos,
        vel,
        acc,
        t,
        pos,
        vel,
        acc,
        tmp_path,
        t_truth=t,
        pos_truth=pos,
        vel_truth=vel,
        acc_truth=acc,
    )
    assert (tmp_path / "TEST_ECEF_overlay_truth.pdf").exists()
