"""Tests for automatic truth path resolution."""

from pathlib import Path


def test_resolve_truth_path(monkeypatch):
    """Ensure ``_resolve_truth_path`` finds ``DATA/TRUTH/STATE_X001.txt`` by default."""
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root / "python" / "src"))
    monkeypatch.delenv("IMU_TRUTH_PATH", raising=False)
    import run_triad_only  # noqa: E402

    expected = root / "DATA" / "TRUTH" / "STATE_X001.txt"
    path = run_triad_only._resolve_truth_path()
    assert Path(path) == expected

