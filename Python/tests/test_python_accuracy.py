import sys
from pathlib import Path
import pytest
np = pytest.importorskip("numpy")
pd = pytest.importorskip("pandas")

from GNSS_IMU_Fusion import main

DATA_DIR = Path(__file__).resolve().parents[1] / "Data"
DATASETS = {
    'X001': (DATA_DIR / "IMU_X001.dat", DATA_DIR / "GNSS_X001.csv"),
    'X002': (DATA_DIR / "IMU_X002.dat", DATA_DIR / "GNSS_X002.csv"),
    'X003': (DATA_DIR / "IMU_X003.dat", DATA_DIR / "GNSS_X002.csv"),
}

@pytest.mark.parametrize("imu_file,gnss_file", DATASETS.values(), ids=DATASETS.keys())
def test_python_accuracy(monkeypatch, imu_file, gnss_file):
    orig_read_csv = pd.read_csv

    def head5000(*args, **kwargs):
        df = orig_read_csv(*args, **kwargs)
        return df.head(5000)

    monkeypatch.setattr(pd, "read_csv", head5000)
    args = ["--imu-file", str(imu_file), "--gnss-file", str(gnss_file),
            "--method", "TRIAD", "--no-plots"]
    monkeypatch.setattr(sys, "argv", ["GNSS_IMU_Fusion.py"] + args)
    main()

    tag = f"{Path(imu_file).stem}_{Path(gnss_file).stem}_TRIAD"
    npz_path = Path("results") / f"{tag}_kf_output.npz"
    data = np.load(npz_path, allow_pickle=True)
    final_pos = data["summary"].item()["final_pos"]
    assert final_pos < 0.05, f"final position error {final_pos:.3f} m >= 0.05 m"
