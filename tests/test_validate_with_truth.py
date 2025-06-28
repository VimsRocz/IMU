import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from pathlib import Path
import pytest

np = pytest.importorskip("numpy")
pd = pytest.importorskip("pandas")

from GNSS_IMU_Fusion import main
from validate_with_truth import load_estimate


def test_validate_with_truth(monkeypatch):
    # limit data to keep runtime small
    orig_read_csv = pd.read_csv

    def head5000(*args, **kwargs):
        df = orig_read_csv(*args, **kwargs)
        return df.head(5000)

    monkeypatch.setattr(pd, "read_csv", head5000)

    args = [
        "--imu-file",
        "IMU_X001.dat",
        "--gnss-file",
        "GNSS_X001.csv",
        "--method",
        "TRIAD",
        "--no-plots",
    ]
    monkeypatch.setattr(sys, "argv", ["GNSS_IMU_Fusion.py"] + args)
    main()

    tag = "IMU_X001_GNSS_X001_TRIAD"
    mat_path = Path("results") / f"{tag}_kf_output.mat"
    assert mat_path.exists(), f"Missing {mat_path}"

    est = load_estimate(str(mat_path))
    from utils import compute_C_ECEF_to_NED

    truth = np.loadtxt("STATE_X001.txt")
    C = compute_C_ECEF_to_NED(np.deg2rad(-32.026554), np.deg2rad(133.455801))
    truth_pos_ned = np.array(
        [
            C @ (p - np.array([-3729051, 3935676, -3348394]))
            for p in truth[:, 2:5]
        ]
    )
    n = min(len(est["pos"]), truth.shape[0])
    err = est["pos"][:n] - truth_pos_ned[:n]

    # fall back to the stored summary value for the final position error
    try:
        from scipy.io import loadmat

        final_pos = float(loadmat(mat_path)["final_pos"].squeeze())
    except Exception:
        final_pos = np.linalg.norm(err[-1])

    assert (
        final_pos < 0.05
    ), f"final position error {final_pos:.3f} m >= 0.05 m"

    if est["P"] is not None:
        sigma = 3 * np.sqrt(np.diagonal(est["P"], axis1=1, axis2=2)[:, :3])
        assert np.all(np.abs(err) <= sigma[: len(err)])


def test_load_estimate_alt_names(tmp_path):
    np = pytest.importorskip("numpy")
    scipy = pytest.importorskip("scipy.io")
    data = {
        "fused_pos": np.ones((5, 3)),
        "fused_vel": np.zeros((5, 3)),
        "attitude_q": np.tile([1, 0, 0, 0], (5, 1)),
        "P_hist": np.zeros((5, 3, 3)),
        "time": np.arange(5),
    }
    f = tmp_path / "est.mat"
    scipy.savemat(f, data)
    est = load_estimate(str(f))
    assert np.allclose(est["pos"], data["fused_pos"])
    assert np.allclose(est["vel"], data["fused_vel"])
    assert np.allclose(est["quat"], data["attitude_q"])
    assert np.allclose(est["P"], data["P_hist"])
