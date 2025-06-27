import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import pytest
from Python.GNSS_IMU_Fusion import main


def test_missing_gnss_file(monkeypatch):
    args = ["--imu-file", "IMU_X001.dat", "--gnss-file", "missing.csv"]
    monkeypatch.setattr("sys.argv", ["Python/GNSS_IMU_Fusion.py"] + args)
    with pytest.raises(FileNotFoundError):
        main()
