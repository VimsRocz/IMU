from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # PYTHON/src -> PYTHON -> ROOT
DATA_DIR   = ROOT / "DATA"
IMU_DIR    = DATA_DIR / "IMU"
GNSS_DIR   = DATA_DIR / "GNSS"
TRUTH_DIR  = DATA_DIR / "Truth"
PY_RES_DIR = ROOT / "PYTHON" / "results"     # Python outputs

def imu_path(name: str) -> Path:
    return IMU_DIR / name

def gnss_path(name: str) -> Path:
    return GNSS_DIR / name

def truth_path(name: str) -> Path:
    return TRUTH_DIR / name

def ensure_results_dir() -> Path:
    """Ensure the Python results directory exists and return its path."""
    PY_RES_DIR.mkdir(parents=True, exist_ok=True)
    return PY_RES_DIR


# Backwards compatibility helper.  Older code imported ``ensure_py_results``
# which simply ensured the directory existed without returning it.  Re-export
# the new helper under that name so existing imports keep working.
def ensure_py_results() -> Path:  # pragma: no cover - compatibility shim
    return ensure_results_dir()
