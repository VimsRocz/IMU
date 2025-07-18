import sys
from pathlib import Path
import pytest
from src.GNSS_IMU_Fusion import main
from src.validate_with_truth import load_estimate
from src.utils import ecef_to_geodetic

np = pytest.importorskip("numpy")
pd = pytest.importorskip("pandas")


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
    from utils import compute_C_ECEF_to_NED, ecef_to_geodetic

    truth = np.loadtxt("STATE_X001.txt")
    ref_ecef = truth[0, 2:5]
    lat_deg, lon_deg, _ = ecef_to_geodetic(*ref_ecef)
    C = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
    truth_pos_ned = np.array([C @ (p - ref_ecef) for p in truth[:, 2:5]])
    n = min(len(est["pos"]), truth.shape[0])
    err = est["pos"][:n] - truth_pos_ned[:n]

    # fall back to the stored summary value for the final position error
    try:
        from scipy.io import loadmat

        final_pos = float(loadmat(mat_path)["final_pos"].squeeze())
    except Exception:
        final_pos = np.linalg.norm(err[-1])

    assert final_pos < 0.05, f"final position error {final_pos:.3f} m >= 0.05 m"

    if est["P"] is not None:
        sigma = 3 * np.sqrt(np.diagonal(est["P"], axis1=1, axis2=2)[:, :3])
        assert np.all(np.abs(err) <= sigma[: len(err)])

    npz_path = Path("results") / f"{tag}_kf_output.npz"
    assert npz_path.exists(), f"Missing {npz_path}"
    npz = np.load(npz_path, allow_pickle=True)
    from scipy.io import loadmat

    mat = loadmat(mat_path)
    for key in ["fused_pos", "fused_vel"]:
        assert key in npz, f"{key} missing from npz"
        assert key in mat, f"{key} missing from mat"

    # verify ECEF Z component matches truth
    z_fused = npz["pos_ecef"][:n, 2]
    z_truth = truth[:n, 4]
    assert np.allclose(z_fused, z_truth, atol=0.05)


@pytest.mark.parametrize(
    "pos_key,vel_key",
    [("fused_pos", "fused_vel"), ("pos_ned", "vel_ned")],
)
def test_load_estimate_alt_names(tmp_path, pos_key, vel_key):
    np = pytest.importorskip("numpy")
    scipy = pytest.importorskip("scipy.io")
    data = {
        pos_key: np.ones((5, 3)),
        vel_key: np.zeros((5, 3)),
        "attitude_q": np.tile([1, 0, 0, 0], (5, 1)),
        "P_hist": np.zeros((5, 3, 3)),
        "time": np.arange(5),
    }
    f = tmp_path / "est.mat"
    scipy.savemat(f, data)
    est = load_estimate(str(f))
    assert np.allclose(est["pos"], data[pos_key])
    assert np.allclose(est["vel"], data[vel_key])
    assert np.allclose(est["quat"], data["attitude_q"])
    assert np.allclose(est["P"], data["P_hist"])


def test_load_estimate_time_s(tmp_path):
    np = pytest.importorskip("numpy")
    scipy = pytest.importorskip("scipy.io")

    data = {
        "fused_pos": np.zeros((5, 3)),
        "fused_vel": np.zeros((5, 3)),
        "attitude_q": np.tile([1, 0, 0, 0], (5, 1)),
        "time_s": np.linspace(0.0, 1.0, 5),
    }

    f = tmp_path / "est.mat"
    scipy.savemat(f, data)

    est = load_estimate(str(f))

    assert np.allclose(est["time"], data["time_s"])


def test_load_estimate_interpolation(tmp_path):
    np = pytest.importorskip("numpy")
    scipy = pytest.importorskip("scipy.io")

    data = {
        "fused_pos": np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
        "fused_vel": np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
        "quat_log": np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
        "time": np.array([0.0, 1.0, 2.0]),
    }
    f = tmp_path / "est.mat"
    scipy.savemat(f, data)

    times = np.linspace(0.0, 2.0, 5)
    est = load_estimate(str(f), times=times)

    expected_pos = np.vstack(
        [np.interp(times, data["time"], data["fused_pos"][:, i]) for i in range(3)]
    ).T
    expected_vel = np.vstack(
        [np.interp(times, data["time"], data["fused_vel"][:, i]) for i in range(3)]
    ).T

    assert np.allclose(est["time"], times)
    assert np.allclose(est["pos"], expected_pos)
    assert np.allclose(est["vel"], expected_vel)
    assert np.allclose(est["quat"], np.tile([1.0, 0.0, 0.0, 0.0], (5, 1)))


def test_load_estimate_quat_interpolation(tmp_path):
    np = pytest.importorskip("numpy")
    scipy = pytest.importorskip("scipy.io")
    from scipy.spatial.transform import Rotation as R, Slerp

    # quaternion rotates 0 -> 45 -> 90 degrees about X
    base_rot = R.from_euler("x", [0.0, 45.0, 90.0], degrees=True)
    quat = base_rot.as_quat()[:, [3, 0, 1, 2]]

    data = {
        "fused_pos": np.zeros((3, 3)),
        "fused_vel": np.zeros((3, 3)),
        "attitude_q": quat,
        "time": np.array([0.0, 1.0, 2.0]),
    }

    f = tmp_path / "est.mat"
    scipy.savemat(f, data)

    times = np.linspace(0.0, 2.0, 5)
    est = load_estimate(str(f), times=times)

    r = R.from_quat(quat[:, [1, 2, 3, 0]])
    slerp = Slerp(data["time"], r)
    expected_q = slerp(times).as_quat()[:, [3, 0, 1, 2]]

    assert np.allclose(est["quat"], expected_q)


def test_overlay_truth_generation(tmp_path, monkeypatch):
    pd = pytest.importorskip("pandas")
    monkeypatch.chdir(tmp_path)

    orig_read_csv = pd.read_csv

    def head_subset(*args, **kwargs):
        df = orig_read_csv(*args, **kwargs)
        return df.head(500)

    monkeypatch.setattr(pd, "read_csv", head_subset)

    repo = Path(__file__).resolve().parents[1]
    args = [
        "--imu-file",
        str(repo / "IMU_X001_small.dat"),
        "--gnss-file",
        str(repo / "GNSS_X001_small.csv"),
        "--method",
        "TRIAD",
    ]
    monkeypatch.setattr(sys, "argv", ["GNSS_IMU_Fusion.py"] + args)
    main()

    est_file = Path("results") / "IMU_X001_small_GNSS_X001_small_TRIAD_kf_output.mat"
    assert est_file.exists()

    from src.validate_with_truth import main as validate_main

    first = np.loadtxt(repo / "STATE_X001_small.txt", comments="#", max_rows=1)
    r0 = first[2:5]
    lat_deg, lon_deg, _ = ecef_to_geodetic(*r0)

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "validate_with_truth.py",
            "--est-file",
            str(est_file),
            "--truth-file",
            str(repo / "STATE_X001_small.txt"),
            "--output",
            str(Path("results")),
            "--ref-lat",
            str(lat_deg),
            "--ref-lon",
            str(lon_deg),
            "--ref-r0",
            str(r0[0]),
            str(r0[1]),
            str(r0[2]),
        ],
    )
    validate_main()

    expected = {
        "TRIAD_NED_overlay_truth.pdf",
        "TRIAD_ECEF_overlay_truth.pdf",
        "TRIAD_Body_overlay_truth.pdf",
    }
    produced = {p.name for p in Path("results").glob("*_overlay_truth.pdf")}
    assert expected.issubset(produced), f"Missing overlays: {expected - produced}"

    # verify raw STATE overlay generation via task6_plot_truth.py
    from src.task6_plot_truth import main as task6_main
    # provide IMU/GNSS files expected by the script
    Path("IMU_X001_small.dat").write_bytes((repo / "IMU_X001_small.dat").read_bytes())
    Path("GNSS_X001_small.csv").write_bytes((repo / "GNSS_X001_small.csv").read_bytes())
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "task6_plot_truth.py",
            "--est-file",
            str(est_file),
            "--truth-file",
            str(repo / "STATE_X001_small.txt"),
            "--output",
            str(Path("results")),
            "--fused-only",
        ],
    )
    task6_main()
    state_files = {p.name for p in Path("results").glob("*_overlay_state.pdf")}
    assert state_files, "Missing state overlay plots"


def test_assemble_frames_small_truth():
    repo = Path(__file__).resolve().parent / "data"
    gnss_file = repo / "simple_gnss.csv"
    truth_file = repo / "simple_truth.txt"

    est = {
        "time": np.array([0.0, 1.0, 2.0]),
        "pos": np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]]),
        "vel": np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.1, 0.0, 0.0]]),
        "quat": np.tile([1.0, 0.0, 0.0, 0.0], (3, 1)),
    }

    from src.validate_with_truth import assemble_frames

    frames = assemble_frames(est, str(gnss_file), str(gnss_file), str(truth_file))

    for frame in ["NED", "ECEF", "Body"]:
        assert "truth" in frames[frame], f"Missing truth in {frame} frame"
        fused = frames[frame]["fused"]
        truth_data = frames[frame]["truth"]
        for f, t in zip(fused, truth_data):
            assert np.shape(f) == np.shape(t)
