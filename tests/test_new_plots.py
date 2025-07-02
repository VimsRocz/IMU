import importlib
import os
import sys
from pathlib import Path
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "src"))

pytest.importorskip("matplotlib")
pd = pytest.importorskip("pandas")

main = importlib.import_module("GNSS_IMU_Fusion").main


def test_body_frame_plots(tmp_path, monkeypatch):
    # run from a temporary working directory so plots are written under tmp_path
    monkeypatch.chdir(tmp_path)

    # limit dataset size for speed
    orig_read_csv = pd.read_csv

    def head_subset(*args, **kwargs):
        df = orig_read_csv(*args, **kwargs)
        return df.head(500)

    monkeypatch.setattr(pd, "read_csv", head_subset)

    repo_root = Path(__file__).resolve().parents[1]
    args = [
        "--imu-file",
        str(repo_root / "IMU_X001_small.dat"),
        "--gnss-file",
        str(repo_root / "GNSS_X001_small.csv"),
        "--method",
        "TRIAD",
    ]
    monkeypatch.setattr(sys, "argv", ["GNSS_IMU_Fusion.py"] + args)
    main()

    res_dir = Path("results")
    expected = [
        "*_task4_all_body.pdf",
        "*_task5_all_body.pdf",
    ]
    for pattern in expected:
        matches = list(res_dir.glob(pattern))
        assert matches, f"Missing plot {pattern}"

    # cleanup
    for f in res_dir.glob("*.pdf"):
        f.unlink()
