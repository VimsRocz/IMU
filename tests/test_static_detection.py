import importlib
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "src"))

np = pytest.importorskip("numpy")
utils = importlib.import_module("utils")
detect_static_interval = utils.detect_static_interval
is_static = utils.is_static


def test_detect_static_interval_basic():
    rng = np.random.default_rng(0)
    static_acc = rng.normal(0, 0.001, size=(500, 3))
    static_gyro = rng.normal(0, 1e-5, size=(500, 3))
    moving_acc = rng.normal(0, 0.1, size=(200, 3))
    moving_gyro = rng.normal(0, 0.01, size=(200, 3))
    accel = np.vstack([static_acc, moving_acc])
    gyro = np.vstack([static_gyro, moving_gyro])
    start, end = detect_static_interval(accel, gyro, window_size=50, min_length=100)
    assert 0 <= start < 50
    assert end > 400


def test_detect_static_interval_longest():
    rng = np.random.default_rng(2)
    static_short_acc = rng.normal(0, 0.001, size=(80, 3))
    static_short_gyro = rng.normal(0, 1e-5, size=(80, 3))
    moving = rng.normal(0, 0.1, size=(200, 3))
    moving_g = rng.normal(0, 0.01, size=(200, 3))
    static_long_acc = rng.normal(0, 0.001, size=(200, 3))
    static_long_gyro = rng.normal(0, 1e-5, size=(200, 3))
    accel = np.vstack([static_short_acc, moving, static_long_acc])
    gyro = np.vstack([static_short_gyro, moving_g, static_long_gyro])
    start, end = detect_static_interval(accel, gyro, window_size=50, min_length=80)
    assert start >= len(static_short_acc) + len(moving) - 1
    assert end - start >= len(static_long_acc)


def test_is_static_true_false():
    rng = np.random.default_rng(1)
    static_acc = rng.normal(0, 0.001, size=(200, 3))
    static_gyro = rng.normal(0, 1e-5, size=(200, 3))
    moving_acc = rng.normal(0, 0.1, size=(200, 3))
    moving_gyro = rng.normal(0, 0.01, size=(200, 3))
    assert is_static(static_acc, static_gyro)
    assert not is_static(moving_acc, moving_gyro)
