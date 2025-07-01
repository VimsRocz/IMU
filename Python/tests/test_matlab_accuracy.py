import subprocess
import shutil
from pathlib import Path

import pytest
np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
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
    res_dir = Path("MATLAB") / "results"
    mat_file = res_dir / "Task5_results_IMU_X001_GNSS_X001.mat"
    assert mat_file.exists(), f"Missing {mat_file}"
    data = scipy.io.loadmat(mat_file)
    x_log = data["x_log"]
    gnss_pos_ned = data["gnss_pos_ned"]
    final_err = np.linalg.norm(x_log[0:3, -1] - gnss_pos_ned[-1, :])
    assert final_err < 0.05, f"final position error {final_err:.3f} m >= 0.05 m"

    assert "vel_log" in data and "accel_from_vel" in data
    vel_log = data["vel_log"]
    accel_from_vel = data["accel_from_vel"]
    gnss_vel_ned = data["gnss_vel_ned"]
    gnss_accel_ned = data["gnss_accel_ned"]
    vel_err = np.linalg.norm(vel_log[:, -1] - gnss_vel_ned[-1, :])
    acc_err = np.linalg.norm(accel_from_vel[:, -1] - gnss_accel_ned[-1, :])
    assert vel_err < 0.05, f"final velocity error {vel_err:.3f} m/s >= 0.05 m/s"
    assert acc_err < 0.05, f"final acceleration error {acc_err:.3f} m/s^2 >= 0.05 m/s^2"

