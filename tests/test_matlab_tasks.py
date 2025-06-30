import subprocess
import shutil
from pathlib import Path

import pytest
np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
import scipy.io


def test_matlab_tasks(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    cmd = (
        "addpath('matlab/pipeline_tasks');"
        "addpath('matlab/helpers');"
        "imu=get_data_file('IMU_X001.dat');"
        "gnss=get_data_file('GNSS_X001.csv');"
        "Task_1(imu, gnss, 'TRIAD');"
        "Task_2(imu, gnss, 'TRIAD');"
        "Task_3(imu, gnss, 'TRIAD');"
        "Task_4(imu, gnss, 'TRIAD');"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)
    mat_file = Path('results/Task3_results_IMU_X001_GNSS_X001.mat')
    assert mat_file.exists(), f"Missing {mat_file}"
    data = scipy.io.loadmat(mat_file, struct_as_record=False, squeeze_me=True)
    task3 = data['task3_results']
    assert hasattr(task3, 'TRIAD')
    R = task3.TRIAD.R
    assert isinstance(R, np.ndarray)
    assert R.shape == (3, 3)
    assert np.isfinite(R).all()

    mat_file4 = Path('results/Task4_results_IMU_X001_GNSS_X001.mat')
    assert mat_file4.exists(), f"Missing {mat_file4}"
    data4 = scipy.io.loadmat(mat_file4)
    assert 'gnss_pos_ned' in data4
    gnss_pos_ned = data4['gnss_pos_ned']
    assert isinstance(gnss_pos_ned, np.ndarray)
    assert gnss_pos_ned.ndim == 2
    assert gnss_pos_ned.shape[1] == 3
    assert np.isfinite(gnss_pos_ned).all()

