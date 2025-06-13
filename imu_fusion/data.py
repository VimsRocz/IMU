import pandas as pd
import numpy as np


def load_gnss_csv(path: str) -> pd.DataFrame:
    """Load GNSS data from a CSV file."""
    return pd.read_csv(path)


def load_imu_dat(path: str) -> np.ndarray:
    """Load IMU data from a whitespace separated dat file."""
    return np.loadtxt(path)
