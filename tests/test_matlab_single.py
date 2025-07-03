import subprocess
import shutil
from pathlib import Path
import pytest
import scipy.io

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")


def test_matlab_single(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    cmd = (
        "imu=get_data_file('IMU_X001_small.dat');"
        "gnss=get_data_file('GNSS_X001_small.csv');"
        "GNSS_IMU_Fusion_single(imu, gnss);"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)

    mat_file = Path('results/IMU_X001_small_GNSS_X001_small_TRIAD_task5_results.mat')
    assert mat_file.exists(), f"Missing {mat_file}"
    data = scipy.io.loadmat(mat_file, struct_as_record=False, squeeze_me=True)
    assert {"gnss_pos_ned", "gnss_vel_ned", "gnss_accel_ned"} <= set(data)
    pos = data["gnss_pos_ned"]
    vel = data["gnss_vel_ned"]
    assert isinstance(pos, np.ndarray) and pos.shape[1] == 3
    assert isinstance(vel, np.ndarray) and vel.shape[1] == 3
    assert np.isfinite(pos).all() and np.isfinite(vel).all()

    res_dir = Path('results')
    for f in res_dir.glob('*.pdf'):
        f.unlink()

