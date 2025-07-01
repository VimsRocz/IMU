import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import pytest

pytest.importorskip("numpy")
pytest.importorskip("pandas")
from GNSS_IMU_Fusion import main


def test_missing_gnss_file(monkeypatch):
    data_dir = Path(__file__).resolve().parents[1] / "Data"
    args = ["--imu-file", str(data_dir / "IMU_X001.dat"), "--gnss-file", "missing.csv"]
    monkeypatch.setattr("sys.argv", ["GNSS_IMU_Fusion.py"] + args)
    with pytest.raises(FileNotFoundError):
        main()
