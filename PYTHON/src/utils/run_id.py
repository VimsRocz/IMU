"""Python counterpart to ``MATLAB/src/utils/run_id.m``."""

from pathlib import Path


def run_id(imu_path: str, gnss_path: str, method: str) -> str:
    """Return consistent run label like ``IMU_X002_GNSS_X002_TRIAD``."""

    imu_tag = Path(imu_path).name.upper().replace(".DAT", "")
    gnss_tag = Path(gnss_path).name.upper().replace(".CSV", "")
    return f"{imu_tag}_{gnss_tag}_{method.upper()}"


__all__ = ["run_id"]

