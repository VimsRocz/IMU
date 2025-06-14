import numpy as np


def estimate_static_bias(accel: np.ndarray, N: int = 1000, alpha: float = 0.2) -> np.ndarray:
    """Return accelerometer bias estimated from the first N samples.

    The raw data is low-pass filtered with a simple IIR filter before taking
    the median.  This improves robustness to outliers when the IMU is first
    powered on.
    """
    if len(accel) == 0:
        return np.zeros(3)
    filtered = np.zeros_like(accel)
    filtered[0] = accel[0]
    for i in range(1, len(accel)):
        filtered[i] = alpha * accel[i] + (1 - alpha) * filtered[i - 1]
    n = min(N, len(filtered))
    return np.median(filtered[:n], axis=0)
