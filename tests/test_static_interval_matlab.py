import subprocess
import shutil
import pytest


def test_static_interval_matlab():
    """Run the MATLAB static interval detection test script."""
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    cmd = "run('tests/test_static_interval_matlab.m');"
    subprocess.run([matlab, "-batch", cmd], check=True)

