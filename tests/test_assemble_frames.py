import numpy as np
from pathlib import Path

from src.validate_with_truth import assemble_frames


def test_assemble_frames_with_truth():
    repo = Path(__file__).resolve().parents[0] / "data"
    gnss_file = repo / "simple_gnss.csv"
    truth_file = repo / "simple_truth.txt"

    est = {
        "time": np.array([0.0, 1.0, 2.0]),
        "pos": np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]]),
        "vel": np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.1, 0.0, 0.0]]),
        "quat": np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
    }

    frames = assemble_frames(est, str(gnss_file), str(truth_file))

    for frame in ["NED", "ECEF", "Body"]:
        assert "truth" in frames[frame], f"Missing truth in {frame} frame"
        fused = frames[frame]["fused"]
        truth = frames[frame]["truth"]
        for f, t in zip(fused, truth):
            assert np.shape(f) == np.shape(t)

