import os
import sys
from pathlib import Path
import builtins

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import pytest

pd = pytest.importorskip("pandas")
from GNSS_IMU_Fusion import main


def test_no_plots_without_cartopy(monkeypatch):
    orig_import = builtins.__import__
    def fake_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name.startswith("cartopy"):
            raise ImportError("No cartopy")
        return orig_import(name, globals, locals, fromlist, level)
    monkeypatch.setattr(builtins, "__import__", fake_import)

    orig_read_csv = pd.read_csv
    def head5000(*args, **kwargs):
        df = orig_read_csv(*args, **kwargs)
        return df.head(5000)
    monkeypatch.setattr(pd, "read_csv", head5000)

    data_dir = Path(__file__).resolve().parents[2] / "Data"
    args = [
        "--imu-file",
        str(data_dir / "IMU_X001.dat"),
        "--gnss-file",
        str(data_dir / "GNSS_X001.csv"),
        "--method",
        "TRIAD",
        "--no-plots",
    ]
    monkeypatch.setattr(sys, "argv", ["GNSS_IMU_Fusion.py"] + args)
    main()

