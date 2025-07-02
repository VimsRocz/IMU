import pytest

from src.GNSS_IMU_Fusion import main


def test_missing_gnss_file(monkeypatch):
    args = ["--imu-file", "IMU_X001.dat", "--gnss-file", "missing.csv"]
    monkeypatch.setattr("sys.argv", ["GNSS_IMU_Fusion.py"] + args)
    with pytest.raises(FileNotFoundError):
        main()
