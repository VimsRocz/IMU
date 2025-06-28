import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import pytest

np = pytest.importorskip("numpy")
scipy = pytest.importorskip("scipy.io")




def test_validate_against_truth_pos_vel_keys(tmp_path, monkeypatch):
    import importlib
    import subprocess

    # Avoid executing the batch processor on import
    def dummy_run(*args, **kwargs):
        return subprocess.CompletedProcess(args[0], 0)

    monkeypatch.setattr(subprocess, "run", dummy_run)
    monkeypatch.setattr(sys, "argv", ["run_triad_only.py"])

    if "run_triad_only" in sys.modules:
        del sys.modules["run_triad_only"]
    rto = importlib.import_module("run_triad_only")

    class DummyRot:
        def __init__(self, arr):
            self.n = len(arr)
        def inv(self):
            return self
        def __mul__(self, other):
            return self
        def magnitude(self):
            return np.zeros(self.n)

    monkeypatch.setattr(rto, "R", type("DummyR", (), {"from_quat": staticmethod(lambda a: DummyRot(a))}))

    # Create minimal estimator output using pos_ned/vel_ned only
    data = {
        "pos_ned": np.zeros((5, 3)),
        "vel_ned": np.zeros((5, 3)),
        "attitude_q": np.tile([1, 0, 0, 0], (5, 1)),
        "time": np.arange(5),
    }
    est_file = tmp_path / "est.mat"
    scipy.savemat(est_file, data)

    truth = np.column_stack([
        np.arange(5),  # count
        np.arange(5, dtype=float),
        np.zeros((5, 3)),
        np.zeros((5, 3)),
        np.tile([1, 0, 0, 0], (5, 1)),
    ])
    truth_file = tmp_path / "truth.txt"
    np.savetxt(truth_file, truth)

    rto.validate_against_truth(truth_file, est_file, results_dir=tmp_path, plot_fname="out.pdf")
    assert (tmp_path / "out.pdf").exists()
