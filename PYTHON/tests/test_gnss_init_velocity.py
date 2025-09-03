import pytest
from src.gnss_imu_fusion.init import compute_reference_vectors
from src.utils.ecef_llh import lla_to_ecef

np = pytest.importorskip("numpy")
pd = pytest.importorskip("pandas")


def test_initial_velocity_conversion(tmp_path):
    # Known latitude/longitude where the transformation is simple
    lat_deg = 0.0
    lon_deg = 0.0
    alt = 0.0
    x, y, z = lla_to_ecef(lat_deg, lon_deg, alt)

    # ECEF velocity pointing toward geographic north (+Z axis)
    vx, vy, vz = 0.0, 0.0, 100.0
    df = pd.DataFrame(
        {
            "X_ECEF_m": [x],
            "Y_ECEF_m": [y],
            "Z_ECEF_m": [z],
            "VX_ECEF_mps": [vx],
            "VY_ECEF_mps": [vy],
            "VZ_ECEF_mps": [vz],
        }
    )
    csv_path = tmp_path / "gnss.csv"
    df.to_csv(csv_path, index=False)

    (
        _,
        _,
        _,
        _,
        _,
        _,
        initial_vel_ned,
        _,
        _,
    ) = compute_reference_vectors(str(csv_path))

    assert np.allclose(initial_vel_ned, np.array([100.0, 0.0, 0.0]), atol=1e-6)
