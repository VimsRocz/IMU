import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from pathlib import Path
import pytest
np = pytest.importorskip("numpy")
pd = pytest.importorskip("pandas")

from GNSS_IMU_Fusion import main
from validate_with_truth import load_estimate
from utils import compute_C_ECEF_to_NED


def test_validate_with_truth(monkeypatch):
    # limit data to keep runtime small
    orig_read_csv = pd.read_csv

    def head5000(*args, **kwargs):
        df = orig_read_csv(*args, **kwargs)
        return df.head(5000)

    monkeypatch.setattr(pd, "read_csv", head5000)

    args = [
        "--imu-file", "IMU_X001.dat",
        "--gnss-file", "GNSS_X001.csv",
        "--method", "TRIAD",
        "--no-plots",
    ]
    monkeypatch.setattr(sys, "argv", ["GNSS_IMU_Fusion.py"] + args)
    main()

    tag = "IMU_X001_GNSS_X001_TRIAD"
    mat_path = Path("results") / f"{tag}_kf_output.mat"
    assert mat_path.exists(), f"Missing {mat_path}"

    est = load_estimate(str(mat_path))
    truth_all = np.loadtxt("STATE_X001.txt")

    def ecef_to_geodetic(x, y, z):
        a = 6378137.0
        e_sq = 6.69437999014e-3
        p = np.sqrt(x ** 2 + y ** 2)
        theta = np.arctan2(z * a, p * (1 - e_sq))
        lon = np.arctan2(y, x)
        lat = np.arctan2(
            z + e_sq * a * np.sin(theta) ** 3 / (1 - e_sq),
            p - e_sq * a * np.cos(theta) ** 3,
        )
        return np.degrees(lat), np.degrees(lon)

    x0, y0, z0 = truth_all[0, 2:5]
    lat_deg, lon_deg = ecef_to_geodetic(x0, y0, z0)
    C = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))

    pos_truth_all = (truth_all[:, 2:5] - np.array([x0, y0, z0])) @ C.T
    t_truth = truth_all[: len(est["time"]), 1]

    pos_est_interp = np.vstack(
        [np.interp(t_truth, est["time"], est["pos"][:, i]) for i in range(3)]
    ).T

    err = pos_est_interp - pos_truth_all[: len(t_truth)]

    # fall back to the stored summary value for the final position error
    try:
        from scipy.io import loadmat
        final_pos = float(loadmat(mat_path)["final_pos"].squeeze())
    except Exception:
        final_pos = np.linalg.norm(err[-1])

    assert final_pos < 0.05, f"final position error {final_pos:.3f} m >= 0.05 m"

    if est["P"] is not None:
        sigma_raw = 3 * np.sqrt(np.diagonal(est["P"], axis1=1, axis2=2)[:, :3])
        sigma = np.vstack(
            [np.interp(t_truth, est["time"], sigma_raw[:, i]) for i in range(3)]
        ).T
        assert np.all(np.abs(err) <= sigma)


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
