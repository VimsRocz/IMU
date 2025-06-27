import subprocess
import shutil
from pathlib import Path

import numpy as np
import pytest
import scipy.io


def test_matlab_accuracy(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    cmd = (
        "imu_path=get_data_file('IMU_X001.dat');"
        "gnss_path=get_data_file('GNSS_X001.csv');"
        "main(imu_path, gnss_path, 'TRIAD');"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)
    mat_file = Path("results/Task5_results_IMU_X001_GNSS_X001.mat")
    assert mat_file.exists(), f"Missing {mat_file}"
    data = scipy.io.loadmat(mat_file)
    x_log = data["x_log"]
    gnss_pos_ned = data["gnss_pos_ned"]
    final_err = np.linalg.norm(x_log[0:3, -1] - gnss_pos_ned[-1, :])
    assert final_err < 0.05, f"final position error {final_err:.3f} m >= 0.05 m"

