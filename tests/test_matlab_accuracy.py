import subprocess
import shutil
from pathlib import Path
import pytest
import scipy.io

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")


def test_matlab_accuracy(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    cmd = (
        "imu=get_data_file('IMU_X001.dat');"
        "gnss=get_data_file('GNSS_X001.csv');"
        "FINAL(imu, gnss, 'TRIAD');"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)

    mat_file = Path('MATLAB/results/IMU_X001_GNSS_X001_TRIAD_final.mat')
    assert mat_file.exists(), f"Missing {mat_file}"
    data = scipy.io.loadmat(mat_file, struct_as_record=False, squeeze_me=True)
    fused_pos = data['fused_pos']
    fused_vel = data['fused_vel']
    summary = data['summary']
    if isinstance(summary, np.ndarray):
        summary = summary.item()
    final_err = float(summary.final_pos)

    assert fused_pos.shape[1] == 3 and fused_vel.shape[1] == 3
    assert np.isfinite(fused_pos).all() and np.isfinite(fused_vel).all()
    assert final_err < 0.05, f"final position error {final_err:.3f} m >= 0.05 m"

