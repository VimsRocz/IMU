import sys
import importlib.util
from pathlib import Path
import types
import runpy
import pytest

pytest.importorskip("numpy")


def test_output_dir_option(tmp_path, monkeypatch):
    repo_root = Path(__file__).resolve().parents[2]
    script = repo_root / "Python" / "run_all_datasets.py"
    spec = importlib.util.spec_from_file_location("run_all_datasets", script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def fake_run_one(*a, **k):
        return ["method=TRIAD rmse_pos=0 final_pos=0"]

    monkeypatch.setattr(mod, "run_one", fake_run_one)
    monkeypatch.setattr(sys, "argv", ["run_all_datasets.py", "--datasets", "X001", "--method", "TRIAD", "--output-dir", str(tmp_path)])

    mod.main()
    assert (tmp_path / "summary.csv").exists()
