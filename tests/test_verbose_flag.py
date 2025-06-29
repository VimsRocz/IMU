import os
import sys
import subprocess
import pytest

pytest.importorskip("cartopy")


def test_verbose_flag_help():
    result = subprocess.run(
        [sys.executable, "GNSS_IMU_Fusion.py", "--verbose", "--help"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert result.returncode == 0
    assert "--verbose" in result.stdout
