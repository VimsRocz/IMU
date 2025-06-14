import numpy as np
from imu_fusion.data import estimate_acc_bias

def test_estimate_acc_bias_constant():
    acc = np.tile([1.0, 2.0, -3.0], (10, 1))
    bias = estimate_acc_bias(acc, n_samples=5)
    assert np.allclose(bias, [1.0, 2.0, -3.0])
