import subprocess
import shutil
import pytest
import scipy.io

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from src.velocity_utils import derive_velocity


def test_derive_velocity_matlab_parity(tmp_path):
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")

    t = np.linspace(0, 5, 51)
    pos = np.vstack([np.sin(t), np.cos(t), t**2]).T

    expected = derive_velocity(t, pos)

    in_mat = tmp_path / "in.mat"
    out_mat = tmp_path / "out.mat"
    scipy.io.savemat(in_mat, {"time_s": t, "pos": pos})
    cmd = (
        f"load('{in_mat}');"
        f"vel = derive_velocity(time_s, pos);"
        f"save('{out_mat}', 'vel');"
    )
    subprocess.run([matlab, "-batch", cmd], check=True)
    data = scipy.io.loadmat(out_mat)
    matlab_vel = data["vel"]

    assert np.allclose(matlab_vel, expected)
