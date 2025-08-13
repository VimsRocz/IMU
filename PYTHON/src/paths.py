from pathlib import Path

# Core resolvers
ROOT = Path(__file__).resolve().parents[2]   # PYTHON/src -> PYTHON -> ROOT
DATA_DIR   = ROOT / "DATA"
IMU_DIR    = DATA_DIR / "IMU"
GNSS_DIR   = DATA_DIR / "GNSS"
TRUTH_DIR  = DATA_DIR / "Truth"
PY_RES_DIR = ROOT / "PYTHON" / "results"     # Python outputs

def repo_root() -> Path:
    """Return the repository root directory."""
    return ROOT

def data_dir() -> Path:
    """Return the repository DATA directory."""
    return DATA_DIR

def imu_path(name_or_dataset: str) -> Path:
    """Return IMU data path.

    Accepts either a full filename like ``IMU_X002.dat`` or a dataset id like
    ``X002`` and builds the correct path under ``DATA/IMU``.
    """
    n = name_or_dataset
    if n.endswith('.dat') or n.startswith('IMU_') or '/' in n or '\\' in n:
        return IMU_DIR / n
    return IMU_DIR / f"IMU_{n}.dat"

def gnss_path(name_or_dataset: str) -> Path:
    """Return GNSS data path.

    Accepts either a full filename like ``GNSS_X002.csv`` or a dataset id like
    ``X002`` and builds the correct path under ``DATA/GNSS``.
    """
    n = name_or_dataset
    if n.endswith('.csv') or n.startswith('GNSS_') or '/' in n or '\\' in n:
        return GNSS_DIR / n
    return GNSS_DIR / f"GNSS_{n}.csv"

def truth_path(name: str = "STATE_X001.txt") -> Path:
    return TRUTH_DIR / name

def python_results_dir() -> Path:
    """Return the PYTHON results directory path (does not create)."""
    return PY_RES_DIR

def ensure_results_dir() -> Path:
    """Ensure the Python results directory exists and return its path."""
    PY_RES_DIR.mkdir(parents=True, exist_ok=True)
    return PY_RES_DIR


def available_imu_files(include_small: bool = False) -> list[str]:
    """Return a sorted list of IMU data files.

    Parameters
    ----------
    include_small : bool, optional
        If ``True``, include files with the ``_small`` suffix.
    """

    return sorted(
        p.name
        for p in IMU_DIR.glob("IMU_*.dat")
        if include_small or "_small" not in p.stem
    )


def available_gnss_files(include_small: bool = False) -> list[str]:
    """Return a sorted list of GNSS data files.

    Parameters
    ----------
    include_small : bool, optional
        If ``True``, include files with the ``_small`` suffix.
    """

    return sorted(
        p.name
        for p in GNSS_DIR.glob("GNSS_*.csv")
        if include_small or "_small" not in p.stem
    )


def available_dataset_ids(include_small: bool = False) -> list[str]:
    """Return dataset identifiers with available IMU or GNSS data.

    Parameters
    ----------
    include_small : bool, optional
        If ``True``, include datasets with the ``_small`` suffix.
    """

    imu_ids = {f[4:-4] for f in available_imu_files(include_small)}
    gnss_ids = {f[5:-4] for f in available_gnss_files(include_small)}
    return sorted(imu_ids | gnss_ids)


# Backwards compatibility helper.  Older code imported ``ensure_py_results``
# which simply ensured the directory existed without returning it.  Re-export
# the new helper under that name so existing imports keep working.
def ensure_py_results() -> Path:  # pragma: no cover - compatibility shim
    return ensure_results_dir()


def normalize_gnss_headers(df):
    """Return a copy of ``df`` with normalised GNSS column headers.

    Column names are stripped of leading/trailing whitespace to avoid
    downstream import errors.  The dataframe is otherwise unchanged.
    """

    return df.rename(columns=lambda c: c.strip())
