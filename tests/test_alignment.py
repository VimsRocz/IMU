import numpy as np, glob, json, pathlib, pytest, re

def _latest_npzs():
    for f in glob.glob("results/*_kf_output.npz"):
        yield pathlib.Path(f).stem, np.load(f, allow_pickle=True)

@pytest.mark.parametrize("tag,data", _latest_npzs())
def test_final_alignment(tag, data, tol=15.0):
    err = data["summary"].item()["final_alignment_error"]
    assert err < tol, f"{tag} drift {err:.2f} m > {tol}"
