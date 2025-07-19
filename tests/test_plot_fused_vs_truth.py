import numpy as np
from src.plot_fused_vs_truth import plot_fused_vs_truth


def test_plot_fused_vs_truth(tmp_path):
    t = np.linspace(0, 1, 5)
    truth = np.vstack([t, t * 0, -t]).T
    fused = truth + 0.1
    out_file = tmp_path / "plot.pdf"
    plot_fused_vs_truth(t, truth, fused, out_file, frame_label="ECEF")
    assert out_file.exists()
