import shutil
from pathlib import Path
import pytest

me = pytest.importorskip("matlab.engine")


def test_triad_engine_runs():
    matlab = shutil.which("matlab")
    if not matlab:
        pytest.skip("MATLAB not available")
    eng = me.start_matlab("-nojvm")
    repo_root = Path(__file__).resolve().parents[2]
    eng.addpath(str(repo_root / "MATLAB"), nargout=0)
    eng.addpath(str(repo_root / "Data"), nargout=0)
    # run with default arguments; returns struct with results
    eng.TRIAD(nargout=1)
    eng.quit()

