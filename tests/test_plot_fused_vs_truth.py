import pytest

np = pytest.importorskip("numpy")
from src.plot_fused_vs_truth import plot_fused_vs_truth


def test_plot_fused_vs_truth(tmp_path):
    t = np.linspace(0, 1, 5)
    truth = np.vstack([t, t * 0, -t]).T
    fused = truth + 0.1
    out_base = tmp_path / "plot"
    plot_fused_vs_truth(t, truth, fused, out_base, frame_label="ECEF")
    assert (out_base.with_suffix(".png")).exists()
