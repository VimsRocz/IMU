import os, sys
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from utils import get_output_dir


def test_get_output_dir_env(monkeypatch, tmp_path):
    monkeypatch.setenv("IMU_OUTPUT_DIR", str(tmp_path))
    out = get_output_dir()
    assert out == tmp_path


def test_get_output_dir_default(monkeypatch):
    monkeypatch.delenv("IMU_OUTPUT_DIR", raising=False)
    expected = Path(__file__).resolve().parents[1] / "results"
    out = get_output_dir()
    assert out == expected
