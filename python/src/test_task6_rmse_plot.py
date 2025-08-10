import pytest

np = pytest.importorskip("numpy")
from task6_overlay_plot import plot_rmse

def test_task6_rmse_plot(tmp_path):
    t = np.linspace(0, 1, 5)
    pos = np.vstack([t, t, t]).T
    vel = pos * 0
    acc = pos * 0
    # Introduce small offset for error
    pos_est = pos + 0.1
    vel_est = vel + 0.1
    acc_est = acc + 0.1

    plot_rmse(
        t,
        pos_est,
        vel_est,
        acc_est,
        pos,
        vel,
        acc,
        "ECEF",
        "TEST",
        "TESTDATA",
        tmp_path,
    )

    assert (tmp_path / "TESTDATA_TEST_Task6_ECEF_RMSE.pdf").exists()

