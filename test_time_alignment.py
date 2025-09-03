#!/usr/bin/env python3
"""Unit test for GNSS/IMU time alignment utilities."""
import os
import sys
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "PYTHON", "src"))

from utils.interp_to import interp_to
from utils.time_utils import compute_time_shift


def test_time_alignment_with_known_offset():
    dt_imu = 0.01
    t_imu = np.arange(0.0, 10.0, dt_imu)
    offset_samples = 30  # known lag in samples

    # Synthetic signal observed by IMU
    imu_series = np.sin(1.3 * t_imu) + 0.5 * np.sin(0.7 * t_imu)
    # GNSS samples are the same signal delayed by ``offset_samples``
    gnss_series = np.roll(imu_series, offset_samples)
    gnss_series[:offset_samples] = imu_series[0]

    lag, t_shift = compute_time_shift(imu_series, gnss_series, dt_imu)
    assert abs(lag) == offset_samples

    # Shift GNSS timestamps by the estimated offset and interpolate onto IMU time
    gnss_on_imu = interp_to(t_imu + t_shift, gnss_series, t_imu)
    _, residual = compute_time_shift(imu_series, gnss_on_imu, dt_imu)
    assert abs(residual) <= dt_imu

