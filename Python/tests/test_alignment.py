import glob
import json
import pathlib
import re
import pytest

np = pytest.importorskip("numpy")

def _latest_npzs():
    for f in glob.glob("results/*_kf_output.npz"):
        yield pathlib.Path(f).stem, np.load(f, allow_pickle=True)

@pytest.mark.parametrize("tag,data", _latest_npzs())
def test_final_alignment(tag, data, tol=15.0):
    summary = data["summary"].item()
    err = summary.get("final_alignment_error", summary.get("final_pos"))
    assert err < tol, f"{tag} drift {err:.2f} m > {tol}"
