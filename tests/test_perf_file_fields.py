import scipy.io
import numpy as np

def test_perf_file_fields(tmp_path):
    perf_file = tmp_path / "IMU_GNSS_bias_and_performance.mat"
    results = {
        "method": "TEST",
        "rmse_pos": 0.1,
        "rmse_vel": 0.2,
        "final_pos_error": 0.0,
        "final_vel_error": 0.0,
        "final_vel": np.zeros(3),
        "final_acc_error": 0.0,
        "accel_bias": np.zeros(3),
        "gyro_bias": np.zeros(3),
        "grav_err_mean": 0.0,
        "grav_err_max": 0.0,
        "omega_err_mean": 0.0,
        "omega_err_max": 0.0,
    }
    scipy.io.savemat(perf_file, {"results": results})
    data = scipy.io.loadmat(perf_file, squeeze_me=True, struct_as_record=False)
    expected = [
        "rmse_pos",
        "rmse_vel",
        "accel_bias",
        "gyro_bias",
        "grav_err_mean",
        "grav_err_max",
        "omega_err_mean",
        "omega_err_max",
    ]
    assert "results" in data, "Missing results struct"
    res = data["results"]
    for field in expected:
        assert hasattr(res, field), f"Missing field {field}"

