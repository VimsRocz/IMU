import subprocess
import shutil

import pytest


def test_matlab_derive_velocity_parity(tmp_path):
    octave = shutil.which("octave")
    if not octave:
        pytest.skip("Octave not available")

    np = pytest.importorskip("numpy")
    spio = pytest.importorskip("scipy.io")

    from src.velocity_utils import derive_velocity as py_derive_velocity

    t = np.linspace(0.0, 10.0, 101)
    pos = np.stack((np.sin(t), np.cos(t), t), axis=1)
    input_mat = tmp_path / "input.mat"
    output_mat = tmp_path / "out.mat"
    spio.savemat(input_mat, {"time_s": t[:, None], "pos": pos})

    cmd = (
        f"addpath('MATLAB'); data=load('{input_mat}'); "
        f"vel=derive_velocity(data.time_s, data.pos); "
        f"save('-mat7-binary', '{output_mat}', 'vel');"
    )
    subprocess.run([octave, "--quiet", "--eval", cmd], check=True)

    vel_matlab = spio.loadmat(output_mat)["vel"]
    vel_py = py_derive_velocity(t, pos)

    assert np.allclose(vel_matlab, vel_py, atol=1e-6)

