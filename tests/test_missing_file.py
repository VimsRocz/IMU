import os, sys
repo_root = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(repo_root, "src"))
sys.path.insert(0, repo_root)
import pytest
from imu_fusion.GNSS_IMU_Fusion import main


def test_missing_gnss_file(monkeypatch):
    args = ["--imu-file", "IMU_X001.dat", "--gnss-file", "missing.csv"]
    monkeypatch.setattr("sys.argv", ["GNSS_IMU_Fusion.py"] + args)
    with pytest.raises(FileNotFoundError):
        main()
