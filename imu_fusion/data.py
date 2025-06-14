import pandas as pd
import numpy as np


def load_gnss_csv(path: str) -> pd.DataFrame:
    """Load GNSS data from a CSV file."""
    return pd.read_csv(path)


def load_imu_dat(path: str) -> np.ndarray:
    """Load IMU data from a whitespace separated dat file."""
    return np.loadtxt(path)


def estimate_acc_bias(acc: np.ndarray, n_samples: int = 4000, alpha: float = 0.1) -> np.ndarray:
    """Estimate accelerometer bias using a low-pass median of initial samples.

    Parameters
    ----------
    acc : np.ndarray
        Raw accelerometer measurements as ``N x 3`` array.
    n_samples : int, optional
        Number of initial samples used for the bias estimate.  The default is
        4000 which corresponds to 10 seconds at 400 Hz.
    alpha : float, optional
        IIR low-pass coefficient.  ``alpha=0`` disables filtering while
        ``alpha=1`` keeps only the newest sample.  A value around ``0.1`` keeps
        the estimate responsive but robust to spikes.

    Returns
    -------
    np.ndarray
        The estimated 3-element bias vector.
    """

    if acc.ndim != 2 or acc.shape[1] != 3:
        raise ValueError("acc must be an N x 3 array")

    n = min(len(acc), n_samples)
    filtered = np.empty((n, 3))
    filtered[0] = acc[0]
    for i in range(1, n):
        filtered[i] = alpha * acc[i] + (1 - alpha) * filtered[i - 1]

    return np.median(filtered, axis=0)
