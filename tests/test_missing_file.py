import importlib
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "src"))

main = importlib.import_module("GNSS_IMU_Fusion").main


def test_missing_gnss_file(monkeypatch):
    args = ["--imu-file", "IMU_X001.dat", "--gnss-file", "missing.csv"]
    monkeypatch.setattr("sys.argv", ["GNSS_IMU_Fusion.py"] + args)
    with pytest.raises(FileNotFoundError):
        main()
