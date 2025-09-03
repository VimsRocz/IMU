import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent / 'PYTHON' / 'src'))
from utils.quaternion import assert_quaternion_convention


def test_assert_quaternion_convention_pass():
    q = np.array([0.0, 0.0, 0.0, 1.0])
    assert_quaternion_convention(q)


def test_assert_quaternion_convention_fail_non_unit():
    q = np.array([0.0, 0.0, 0.0, 2.0])
    with pytest.raises(AssertionError):
        assert_quaternion_convention(q)
