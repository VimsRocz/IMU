"""Python counterpart to ``MATLAB/src/utils/run_id.m``."""

from pathlib import Path


def run_id(imu_path: str, gnss_path: str, method: str) -> str:
    """Return a standardised run identifier ``IMU_<id>_GNSS_<id>_<METHOD>``."""

    imu_name = Path(imu_path).stem
    gnss_name = Path(gnss_path).stem

    if not imu_name.lower().startswith("imu_"):
        imu_name = f"IMU_{imu_name}"
    if not gnss_name.lower().startswith("gnss_"):
        gnss_name = f"GNSS_{gnss_name}"

    return f"{imu_name}_{gnss_name}_{method.upper()}"


__all__ = ["run_id"]

