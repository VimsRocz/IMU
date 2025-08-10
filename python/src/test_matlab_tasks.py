import subprocess
import shutil
from pathlib import Path
import pytest
import scipy.io

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")


def test_matlab_tasks(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    cmd = (
        "TRIAD('IMU_X001.dat', 'GNSS_X001.csv');"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)

    mat_file = Path('results/IMU_X001_GNSS_X001_TRIAD_output.mat')
    assert mat_file.exists(), f"Missing {mat_file}"
    data = scipy.io.loadmat(mat_file, struct_as_record=False, squeeze_me=True)
    assert {'pos', 'vel', 'q', 'summary'} <= set(data)
    pos = data['pos']
    vel = data['vel']
    q = data['q']
    assert isinstance(pos, np.ndarray) and pos.shape[1] == 3
    assert isinstance(vel, np.ndarray) and vel.shape[1] == 3
    assert isinstance(q, np.ndarray) and q.size == 4
    assert np.isfinite(pos).all() and np.isfinite(vel).all() and np.isfinite(q).all()

