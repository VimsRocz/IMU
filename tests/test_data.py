import numpy as np
from imu_fusion.data import estimate_acc_bias, apply_acc_bias

def test_estimate_acc_bias_constant():
    acc = np.tile([1.0, 2.0, -3.0], (10, 1))
    bias = estimate_acc_bias(acc, n_samples=5)
    assert np.allclose(bias, [1.0, 2.0, -3.0])


def test_apply_acc_bias():
    acc = np.array([[2.0, 2.0, 5.0]])
    bias = np.array([1.0, -1.0, 2.0])
    corrected = apply_acc_bias(acc, bias)
    assert np.allclose(corrected, [[1.0, 3.0, 3.0]])
