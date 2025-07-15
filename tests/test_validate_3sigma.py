import sys
from pathlib import Path
import pytest

from src.validate_3sigma import main

np = pytest.importorskip("numpy")


def test_no_overlap_error(tmp_path, monkeypatch):
    data = {
        "time": np.array([10.0, 11.0, 12.0]),
        "pos": np.zeros((3, 3)),
        "vel": np.zeros((3, 3)),
        "quat": np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
    }
    est_file = tmp_path / "est.npz"
    np.savez(est_file, **data)
    truth_file = Path("tests/data/simple_truth.txt")

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "validate_3sigma.py",
            "--est-file",
            str(est_file),
            "--truth-file",
            str(truth_file),
        ],
    )
    with pytest.raises(RuntimeError) as excinfo:
        main()
    msg = str(excinfo.value)
    assert "estimate spans 10.00-12.00s" in msg
    assert "truth spans 0.00-2.00s" in msg
    assert "time-shift" in msg


def test_time_shift_allows_overlap(tmp_path, monkeypatch):
    data = {
        "time": np.array([3.0, 4.0, 5.0]),
        "pos": np.zeros((3, 3)),
        "vel": np.zeros((3, 3)),
        "quat": np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
    }
    est_file = tmp_path / "est.npz"
    np.savez(est_file, **data)
    truth_file = Path("tests/data/simple_truth.txt")

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "validate_3sigma.py",
            "--est-file",
            str(est_file),
            "--truth-file",
            str(truth_file),
            "--output-dir",
            str(tmp_path),
            "--time-shift",
            "3.0",
        ],
    )

    main()
    assert (tmp_path / "pos_err_X.pdf").exists()
