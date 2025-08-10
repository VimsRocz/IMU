"""Locate a data file within the repository.

Searches the DATA/IMU, DATA/GNSS, and DATA/TRUTH directories then the
repository root, while accepting absolute paths or relative paths.
This mirrors MATLAB's ``ensure_input_file`` utility.

Usage
-----
    ensure_input_file('IMU_X002.dat')
"""

from pathlib import Path


def _search_roots() -> list[Path]:
    """Return candidate directories for dataset search."""
    repo = Path(__file__).resolve().parents[3]
    data = repo / "DATA"
    return [
        repo,  # legacy root
        data / "IMU",
        data / "GNSS",
        data / "TRUTH",
    ]


def ensure_input_file(path_like: str) -> Path:
    """Resolve *path_like* to an existing dataset path.

    Parameters
    ----------
    path_like : str
        Absolute path, relative path, or bare filename.

    Returns
    -------
    Path
        Absolute path to the dataset.
    """
    p = Path(path_like)
    if p.is_absolute() and p.exists():
        return p

    # try as-is relative to current working directory
    if p.exists():
        return p.resolve()

    # search standard roots
    for root in _search_roots():
        candidate = root / p.name
        if candidate.exists():
            return candidate.resolve()

    raise FileNotFoundError(f"Dataset not found: {path_like}")
