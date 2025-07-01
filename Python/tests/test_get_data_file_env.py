import os, sys
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from utils import get_data_file


def test_get_data_file_env(monkeypatch, tmp_path):
    f = tmp_path / "test.txt"
    f.write_text("data")
    monkeypatch.setenv("IMU_DATA_PATH", str(tmp_path))
    assert get_data_file("test.txt") == f
