import os
import subprocess
import shutil
from pathlib import Path
import pytest
pytest.importorskip("scipy")
import scipy.io


def test_pipeline_smoke(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    # Run MATLAB pipeline from repo root using full paths
    cmd = (
        "imu_path=get_data_file('IMU_X001.dat');"
        "gnss_path=get_data_file('GNSS_X001.csv');"
        "main(imu_path, gnss_path);"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)
    out4 = Path("MATLAB/results") / "task4_results.mat"
    out5 = Path("MATLAB/results") / "task5_results.mat"
    assert out4.exists(), f"Missing {out4}"
    assert out5.exists(), f"Missing {out5}"
    data4 = scipy.io.loadmat(out4)
    data5 = scipy.io.loadmat(out5)
    for data in (data4, data5):
        assert "gnss_pos_ned" in data
        assert "gnss_vel_ned" in data
