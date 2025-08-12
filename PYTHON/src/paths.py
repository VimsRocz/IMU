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


# ---------------------------------------------------------------------------
# Dataset discovery helpers
# ---------------------------------------------------------------------------

def _filter_small(names):
    """Return ``names`` without entries containing ``'small'``."""
    return sorted(n for n in names if "small" not in n)


def available_imu_files(include_small: bool = False):
    """Return a sorted list of IMU data files."""
    names = [p.name for p in IMU_DIR.glob("IMU_*.dat")]
    return sorted(names) if include_small else _filter_small(names)


def available_gnss_files(include_small: bool = False):
    """Return a sorted list of GNSS data files."""
    names = [p.name for p in GNSS_DIR.glob("GNSS_*.csv")]
    return sorted(names) if include_small else _filter_small(names)


def available_dataset_ids():
    """Return dataset IDs present in both IMU and GNSS directories.

    Dataset IDs are inferred from filenames like ``IMU_X001.dat`` and
    ``GNSS_X001.csv``.  Files containing ``'small'`` are ignored.
    """

    imu_ids = {n.split("_")[1].split(".")[0] for n in available_imu_files()}
    gnss_ids = {n.split("_")[1].split(".")[0] for n in available_gnss_files()}
    return sorted(imu_ids & gnss_ids)


def default_dataset_pairs():
    """Return list of ``(imu_file, gnss_file)`` pairs discovered at runtime.

    Each IMU file is paired with the GNSS file sharing the same dataset ID.  If
    no matching GNSS file exists, the last GNSS file in sorted order is used as
    a fallback (mirroring historical behaviour for ``X003``).
    """

    imu_files = available_imu_files()
    gnss_files = available_gnss_files()
    if not gnss_files:
        return [(imu, "") for imu in imu_files]

    gnss_lookup = {n.split("_")[1].split(".")[0]: n for n in gnss_files}
    fallback = gnss_files[-1]
    pairs = []
    for imu in imu_files:
        ds_id = imu.split("_")[1].split(".")[0]
        gnss = gnss_lookup.get(ds_id, fallback)
        pairs.append((imu, gnss))
    return pairs

