import os
import sys
import types
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

np = pytest.importorskip("numpy")
pd = pytest.importorskip("pandas")

plots = types.ModuleType("plots")
plots.plot_frame = lambda *a, **k: None
sys.modules.setdefault("plots", plots)

from validate_with_truth import prepare_truth_frames, assemble_frames
from utils import ecef_to_geodetic, compute_C_ECEF_to_NED


def test_truth_alignment():
    data_dir = Path(__file__).resolve().parents[1] / "Data"
    gnss = pd.read_csv(data_dir / "GNSS_X001.csv")
    start = gnss["Posix_Time"].iloc[0]
    gnss = gnss[gnss["Posix_Time"] - start <= 2]

    t_g = gnss["Posix_Time"].to_numpy() - start
    pos_ecef = gnss[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].to_numpy()
    vel_ecef = gnss[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].to_numpy()

    lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_ecef[0])
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = pos_ecef[0]
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)

    fused_pos = np.array([C @ (p - ref_r0) for p in pos_ecef])
    fused_vel = np.array([C @ v for v in vel_ecef])
    est = {"time": t_g, "pos": fused_pos, "vel": fused_vel}

    frames = assemble_frames(est, str(data_dir / "IMU_X001.dat"), str(data_dir / "GNSS_X001.csv"), ref_lat, ref_lon, ref_r0)

    truth = np.loadtxt(data_dir / "STATE_X001.txt")
    truth = truth[truth[:, 1] <= 2]
    truth_frames = prepare_truth_frames(truth, ref_lat, ref_lon, ref_r0, frames["NED"]["fused"][0])

    err = np.linalg.norm(frames["NED"]["fused"][1] - truth_frames["NED"][1], axis=1)
    assert np.max(err) < 5.0
