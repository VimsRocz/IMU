"""Python counterpart to ``MATLAB/src/utils/run_id.m``."""

from pathlib import Path


def run_id(imu_path: str, gnss_path: str, method: str) -> str:
    """Return consistent run label like ``IMU_X002_GNSS_X002_TRIAD``."""

    # Use .stem to match the TAG function in GNSS_IMU_Fusion.py
    # This preserves the original case (e.g., IMU_X002_small not IMU_X002_SMALL)
    imu_tag = Path(imu_path).stem
    gnss_tag = Path(gnss_path).stem
    return f"{imu_tag}_{gnss_tag}_{method.upper()}"


__all__ = ["run_id"]

