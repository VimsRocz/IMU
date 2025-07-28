import sys
from pathlib import Path
import pytest
import numpy as np

import validate_and_plot


def test_validate_and_plot_missing_field(tmp_path, monkeypatch):
    est_data = {
        "time": np.array([0.0, 1.0, 2.0]),
        "quat": np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
    }
    est_file = tmp_path / "est.npz"
    np.savez(est_file, **est_data)

    truth_file = Path("tests/data/simple_truth.txt")
    out_dir = tmp_path / "out"

    args = [
        "--est-file",
        str(est_file),
        "--truth-file",
        str(truth_file),
        "--output-dir",
        str(out_dir),
    ]
    monkeypatch.setattr(sys, "argv", ["validate_and_plot.py"] + args)
    with pytest.raises(KeyError):
        validate_and_plot.main()
