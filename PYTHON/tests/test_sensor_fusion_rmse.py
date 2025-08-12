import shutil
from pathlib import Path
import pytest

from src.compare_python_matlab import (
    run_python_pipeline,
    run_matlab_pipeline,
    load_python_metrics,
    load_matlab_metrics,
)

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")


def test_sensor_fusion_rmse_parity():
    matlab = shutil.which("matlab") or shutil.which("octave")
    if not matlab:
        pytest.skip("MATLAB/Octave not available")

    repo_root = Path(__file__).resolve().parents[1]
    imu_file = repo_root / "IMU_X001_small.dat"
    gnss_file = repo_root / "GNSS_X001_small.csv"
    method = "TRIAD"

    py_mat = run_python_pipeline(str(imu_file), str(gnss_file), method)
    assert py_mat.exists(), f"Missing {py_mat}"
    py_rmse, _ = load_python_metrics(py_mat)

    mat_mat = run_matlab_pipeline(str(imu_file), str(gnss_file), method)
    assert mat_mat.exists(), f"Missing {mat_mat}"
    mat_rmse, _ = load_matlab_metrics(mat_mat, str(imu_file), str(gnss_file))

    expected_rmse = 0.29
    assert py_rmse == pytest.approx(expected_rmse, rel=0.15, abs=0.05)
    assert mat_rmse == pytest.approx(expected_rmse, rel=0.15, abs=0.05)
    assert py_rmse == pytest.approx(mat_rmse, rel=0.05, abs=0.01)
