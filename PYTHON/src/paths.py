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

def ensure_py_results() -> None:
    PY_RES_DIR.mkdir(parents=True, exist_ok=True)
