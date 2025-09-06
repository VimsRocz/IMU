import re
from pathlib import Path


def _tag_id(p):
    """Return Xnnn id from a path-like, or None if absent."""
    if p is None:
        return None
    m = re.search(r"X(\d{3})", Path(p).stem, flags=re.IGNORECASE)
    return m.group(1) if m else None


def assert_single_pair(imu_path, gnss_path, truth_path, *, force_mix: bool = False):
    """Ensure IMU/GNSS/Truth belong to the same dataset id (Xnnn).

    Parameters
    ----------
    imu_path, gnss_path, truth_path : str | Path | None
        File paths containing an ``Xnnn`` dataset tag in their stem.
    force_mix : bool
        When True, do not raise on mismatches (use with care).

    Returns
    -------
    str | None
        The detected dataset id, if any.
    """
    i, g, t = map(_tag_id, (imu_path, gnss_path, truth_path))
    if not force_mix and not (i and g and t and i == g == t):
        raise ValueError(
            f"[PairGuard] Mismatched dataset IDs: IMU=X{i}, GNSS=X{g}, TRUTH=X{t}. "
            f"Use the same version (e.g., all X001)."
        )
    return i or g or t

