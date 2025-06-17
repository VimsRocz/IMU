import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import numpy as np
from utils import adaptive_zupt_threshold, save_static_zupt_params


def test_adaptive_zupt_threshold_constant():
    accel = np.ones((500, 3))
    gyro = np.ones((500, 3)) * 0.5
    a_t, g_t = adaptive_zupt_threshold(accel, gyro, factor=2.0)
    assert np.isclose(a_t, np.sqrt(3))
    assert np.isclose(g_t, np.sqrt(3) * 0.5)


def test_save_static_zupt_params(tmp_path):
    f = tmp_path / "params.txt"
    save_static_zupt_params(f, 10, 20, 3, 0.4)
    content = f.read_text()
    assert "Static start: 10" in content
    assert "Static end: 20" in content
    assert "Static length: 10" in content
    assert "ZUPT threshold: 0.4" in content
    assert "Total ZUPT events: 3" in content
