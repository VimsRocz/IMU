import argparse
import logging
import sys
import os
from pathlib import Path


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from filterpy.kalman import KalmanFilter
from scipy.signal import butter, filtfilt
from typing import Tuple

from scripts.plot_utils import save_plot, plot_attitude
from utils import (
    detect_static_interval,
    is_static,
    compute_C_ECEF_to_NED,
    ecef_to_geodetic,
)
from scripts.validate_filter import compute_residuals, plot_residuals
from scipy.spatial.transform import Rotation as R

try:
    from rich.console import Console
except ImportError:
    logging.error("\u2757  Missing dependency: install with `pip install rich`")
    sys.exit(1)

try:
    console = Console()
    log = console.log
except Exception:
    logging.basicConfig(level=logging.INFO)
    log = logging.info
TAG = "{imu}_{gnss}_{method}".format  # helper

# Colour palette for plotting per attitude-initialisation method
COLORS = {
    "TRIAD": "tab:blue",
    "Davenport": "tab:orange",
    "SVD": "tab:green",
}

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
from gnss_imu_fusion.init_vectors import (
    average_rotation_matrices,
    svd_alignment,
    butter_lowpass_filter,
    angle_between,
    compute_wahba_errors,
)
from gnss_imu_fusion.plots import (
    save_zupt_variance,
    save_euler_angles,
    save_residual_plots,
    save_attitude_over_time,
)
from gnss_imu_fusion.kalman_filter import (
    quat_multiply,
    quat_from_rate,
    quat2euler,
)

# Minimum number of samples required from a static interval for bias estimation
MIN_STATIC_SAMPLES = 500


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu-file", required=True)
    parser.add_argument("--gnss-file", required=True)
    parser.add_argument(
        "--method",
        default="Davenport",
        choices=["TRIAD", "SVD", "Davenport"],
    )
    parser.add_argument("--mag-file", help="CSV file with magnetometer data")
    parser.add_argument(
        "--use-gnss-heading",
        action="store_true",
        help="Use initial GNSS velocity for yaw if no magnetometer",
    )
    parser.add_argument("--accel-noise", type=float, default=0.1)
    parser.add_argument("--accel-bias-noise", type=float, default=1e-5)
    parser.add_argument("--gyro-bias-noise", type=float, default=1e-5)
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip matplotlib savefig to speed up CI runs",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose debug output",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    method = args.method
    gnss_file = args.gnss_file
    imu_file = args.imu_file

    os.makedirs("results", exist_ok=True)

    imu_stem = Path(args.imu_file).stem
    gnss_stem = Path(args.gnss_file).stem
    tag = TAG(imu=imu_stem, gnss=gnss_stem, method=method)
    summary_tag = f"{imu_stem}_{gnss_stem}"

    logging.info(f"Running attitude-estimation method: {method}")
    
    if not os.path.exists(gnss_file):
        logging.error(f"GNSS file not found: {gnss_file}")
        raise FileNotFoundError(f"{gnss_file} not found")
    if not os.path.exists(imu_file):
        logging.error(f"IMU file not found: {imu_file}")
        raise FileNotFoundError(f"{imu_file} not found")
    
    
    # ================================
    # TASK 1: Define Reference Vectors in NED Frame
    # ================================
    logging.info("TASK 1: Define reference vectors in NED frame")
    
    # --------------------------------
    # Subtask 1.1: Set Initial Latitude and Longitude from GNSS ECEF Data
    # --------------------------------
    logging.info("Subtask 1.1: Setting initial latitude and longitude from GNSS ECEF data.")
    try:
        gnss_data = pd.read_csv(gnss_file)
    except FileNotFoundError:
        logging.error(f"GNSS file not found: {gnss_file}")
        raise
    except Exception as e:
        logging.error(f"Failed to load GNSS data file: {e}")
        raise
    
    # Debug: Print column names and first few rows of ECEF coordinates
    logging.debug("GNSS data columns: %s", gnss_data.columns.tolist())
    logging.debug("First few rows of ECEF coordinates:\n%s", gnss_data[['X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m']].head())
    
    # Find first row with non-zero ECEF coordinates
    valid_rows = gnss_data[(gnss_data['X_ECEF_m'] != 0) | (gnss_data['Y_ECEF_m'] != 0) | (gnss_data['Z_ECEF_m'] != 0)]
    if not valid_rows.empty:
        initial_row = valid_rows.iloc[0]
        x_ecef = float(initial_row['X_ECEF_m'])
        y_ecef = float(initial_row['Y_ECEF_m'])
        z_ecef = float(initial_row['Z_ECEF_m'])
        lat_deg, lon_deg, alt = ecef_to_geodetic(x_ecef, y_ecef, z_ecef)
        lat = np.deg2rad(lat_deg)
        lon = np.deg2rad(lon_deg)
        initial_vel = initial_row[['VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps']].values
        C_e2n_init = compute_C_ECEF_to_NED(lat, lon)
        initial_vel_ned = C_e2n_init @ initial_vel
        logging.info(f"Computed initial latitude: {lat_deg:.6f}°, longitude: {lon_deg:.6f}° from ECEF coordinates.")
        logging.debug(f"Initial latitude: {lat_deg:.6f}°, Initial longitude: {lon_deg:.6f}°")
    else:
        raise ValueError("No valid ECEF coordinates found in GNSS data.")
    
    # --------------------------------
    # Subtask 1.2: Define Gravity Vector in NED
    # --------------------------------
    logging.info("Subtask 1.2: Defining gravity vector in NED frame.")
    
    # Gravity vector in NED frame: g_NED = [0, 0, g], where g ≈ 9.81 m/s²
    g = 9.81
    g_NED = np.array([0.0, 0.0, g])
    logging.info(f"Gravity vector in NED: {g_NED} m/s^2 (Down positive, magnitude g = {g:.2f} m/s^2)")
    
    # --------------------------------
    # Subtask 1.3: Define Earth Rotation Rate Vector in NED
    # --------------------------------
    logging.info("Subtask 1.3: Defining Earth rotation rate vector in NED frame.")
    
    # Earth rotation rate in NED frame: ω_ie,NED = ω_E * [cos(φ), 0, -sin(φ)]
    omega_E = 7.2921159e-5
    omega_ie_NED = omega_E * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    logging.info(f"Earth rotation rate in NED: {omega_ie_NED} rad/s (North, East, Down)")

    mag_NED = None
    if args.mag_file:
        try:
            from datetime import date
            import geomag.geomag as gm
            gm_model = gm.GeoMag()
            res = gm_model.GeoMag(lat_deg, lon_deg, alt, date.today())
            mag_NED = np.array([res.bx, res.by, res.bz])
            logging.info(f"Magnetic field in NED: {mag_NED} nT")
        except Exception as e:
            logging.error(f"Failed to compute magnetic field: {e}")
    
    # --------------------------------
    # Subtask 1.4: Validate and Print Reference Vectors
    # --------------------------------
    logging.info("Subtask 1.4: Validating reference vectors.")
    
    # Validate vector shapes and components
    assert g_NED.shape == (3,), "Gravity vector must be a 3D vector."
    assert omega_ie_NED.shape == (3,), "Earth rotation rate vector must be a 3D vector."
    assert np.isclose(g_NED[0], 0) and np.isclose(g_NED[1], 0), "Gravity should have no North/East component."
    assert np.isclose(omega_ie_NED[1], 0), "Earth rate should have no East component in NED."
    logging.info("Reference vectors validated successfully.")
    
    # Print reference vectors
    logging.info("==== Reference Vectors in NED Frame ====")
    logging.info(f"Gravity vector (NED):        {g_NED} m/s^2")
    logging.info(f"Earth rotation rate (NED):   {omega_ie_NED} rad/s")
    logging.info(f"Latitude (deg):              {lat_deg:.6f}")
    logging.info(f"Longitude (deg):             {lon_deg:.6f}")
    
    # --------------------------------
    # Subtask 1.5: Plot Location on Earth Map
    # --------------------------------
    logging.info("Subtask 1.5: Plotting location on Earth map.")
    if not args.no_plots:
        try:
            import cartopy.crs as ccrs
        except Exception as e:
            logging.warning(f"cartopy not available, skipping map generation: {e}")
        else:
            # Create figure with PlateCarree projection
            fig = plt.figure(figsize=(10, 5))
            ax = fig.add_subplot(1, 1, 1, projection=ccrs.PlateCarree())
            ax.stock_img()  # Add Earth background image

            # Set map extent to focus on the location
            ax.set_extent([lon_deg - 5, lon_deg + 5, lat_deg - 5, lat_deg + 5], crs=ccrs.PlateCarree())

            # Plot the initial location with a red marker
            ax.plot(lon_deg, lat_deg, 'ro', markersize=10, transform=ccrs.PlateCarree())
            ax.text(
                lon_deg + 1,
                lat_deg,
                f"Lat: {lat_deg:.4f}°, Lon: {lon_deg:.4f}°",
                transform=ccrs.PlateCarree(),
            )

            # Set plot title and save
            plt.title("Initial Location on Earth Map")
            plt.savefig(f"results/{tag}_location_map.pdf")
            plt.close()
            logging.info("Location map saved")
    else:
        logging.info("Skipping plot generation (--no-plots)")
    
    
    # ================================
    # TASK 2: Measure the Vectors in the Body Frame
    # ================================
    logging.info("TASK 2: Measure the vectors in the body frame")
    
    # --------------------------------
    # Subtask 2.1: Load and Parse IMU Data
    # --------------------------------
    logging.info("Subtask 2.1: Loading and parsing IMU data.")
    try:
        data = np.loadtxt(imu_file)
    except FileNotFoundError:
        logging.error(f"IMU file not found: {imu_file}")
        raise
    except Exception as e:
        logging.error(f"Failed to load IMU data file: {e}")
        raise

    if data.shape[1] >= 10:
        acc = data[:, 5:8]  # Velocity increments (m/s)
        gyro = data[:, 2:5]  # Angular increments (rad)
    else:
        logging.error(f"Unexpected data format in {imu_file}.")
        raise ValueError("Invalid IMU data format.")

    logging.info(f"IMU data loaded: {data.shape[0]} samples")
    logging.debug(f"IMU data loaded: {data.shape[0]} samples")

    # Estimate IMU sampling period from time column if available
    if data.shape[0] > 1:
        dt_imu = np.mean(np.diff(data[:100, 1]))
    else:
        dt_imu = 1.0 / 400.0
    if dt_imu <= 0:
        dt_imu = 1.0 / 400.0
    logging.info(f"Estimated IMU sampling period: {dt_imu:.6f} s")
    
    # --------------------------------
    # Subtask 2.2: Estimate Static Body-Frame Vectors
    # --------------------------------
    logging.info(
        "Subtask 2.2: Estimating static body-frame vectors using a low-motion interval."
    )

    # Convert increments to rates
    acc = acc / dt_imu  # m/s^2
    gyro = gyro / dt_imu  # rad/s

    # Low-pass filter to suppress high-frequency noise
    acc = butter_lowpass_filter(acc)
    gyro = butter_lowpass_filter(gyro)

    # --- Detect a static interval automatically using variance thresholds ---
    start_idx, end_idx = detect_static_interval(
        acc,
        gyro,
        window_size=80,
        accel_var_thresh=0.01,
        gyro_var_thresh=1e-6,
        min_length=80,
    )
    N_static = end_idx - start_idx
    static_acc = np.mean(acc[start_idx:end_idx], axis=0)
    static_gyro = np.mean(gyro[start_idx:end_idx], axis=0)
    acc_var = np.var(acc[start_idx:end_idx], axis=0)
    gyro_var = np.var(gyro[start_idx:end_idx], axis=0)
    start_t = start_idx * dt_imu
    end_t = end_idx * dt_imu
    logging.info(
        f"Static interval: {start_idx} to {end_idx} (length {N_static} samples)"
    )
    logging.info(
        f"Static accelerometer vector (mean over {N_static} samples): {static_acc}"
    )
    logging.info(
        f"Static gyroscope vector (mean over {N_static} samples): {static_gyro}"
    )
    logging.info(
        f"Window time: {start_t:.3f}s to {end_t:.3f}s | "
        f"Accel var: {acc_var} | Gyro var: {gyro_var}"
    )

    with open("triad_init_log.txt", "a") as logf:
        logf.write(
            f"{imu_file}: window {start_idx}-{end_idx} (t={start_t:.3f}-{end_t:.3f}s)\n"
        )
        logf.write(f"acc_mean={static_acc} acc_var={acc_var}\n")
        logf.write(f"gyro_mean={static_gyro} gyro_var={gyro_var}\n")
    g_norm = np.linalg.norm(static_acc)
    omega_norm = np.linalg.norm(static_gyro)
    logging.info(f"Estimated gravity magnitude from IMU: {g_norm:.4f} m/s² (expected ~9.81)")
    logging.info(f"Estimated Earth rotation magnitude from IMU: {omega_norm:.6e} rad/s (expected ~7.2921e-5)")

    # Simple accelerometer scale calibration
    scale_factor = 9.81 / g_norm if g_norm > 0 else 1.0
    if abs(scale_factor - 1.0) > 0.05:
        logging.info(f"Applying accelerometer scale factor: {scale_factor:.4f}")
    acc *= scale_factor
    static_acc *= scale_factor
    g_norm *= scale_factor
    logging.info(f"Static accelerometer mean: {static_acc}")
    logging.info(f"Static gyroscope mean: {static_gyro}")
    logging.info(f"Gravity magnitude: {g_norm:.4f} m/s²")
    logging.info(f"Earth rotation magnitude: {omega_norm:.6e} rad/s")
    
    # --------------------------------
    # Subtask 2.3: Define Gravity and Earth Rate in Body Frame
    # --------------------------------
    logging.info("Subtask 2.3: Defining gravity and Earth rotation rate in the body frame.")
    g_body = -static_acc
    omega_ie_body = static_gyro
    logging.info(f"Gravity vector in body frame (g_body): {g_body} m/s^2")
    logging.info(f"Earth rotation rate in body frame (omega_ie_body): {omega_ie_body} rad/s")
    logging.info(f"Gravity vector (g_body): {g_body} m/s^2")
    logging.info(f"Earth rotation rate (omega_ie_body): {omega_ie_body} rad/s")

    mag_body = None
    if args.mag_file:
        try:
            mag_data = np.loadtxt(args.mag_file, delimiter=",")
        except Exception as e:
            logging.error(f"Failed to load magnetometer file: {e}")
            mag_data = None
        if mag_data is not None and mag_data.ndim == 2 and mag_data.shape[1] >= 3:
            mag_data = butter_lowpass_filter(mag_data[:, :3])
            m_start, m_end = detect_static_interval(mag_data, mag_data, window_size=80)
            mag_body = np.mean(mag_data[m_start:m_end], axis=0)
            logging.info(f"Static magnetometer vector: {mag_body}")
    
    # --------------------------------
    # Subtask 2.4: Validate and Print Body-Frame Vectors
    # --------------------------------
    logging.info("Subtask 2.4: Validating measured vectors in the body frame.")
    expected_omega = 7.2921159e-5
    assert g_body.shape == (3,), "g_body must be a 3D vector."
    assert omega_ie_body.shape == (3,), "omega_ie_body must be a 3D vector."
    g_norm = np.linalg.norm(g_body)
    omega_norm = np.linalg.norm(omega_ie_body)
    if g_norm < 0.1 * 9.81:
        logging.warning("Gravity magnitude is very low; check accelerometer or static assumption.")
    if omega_norm < 0.5 * expected_omega:
        logging.warning("Earth rotation rate is low; check gyroscope or static assumption.")
    logging.info(f"Magnitude of g_body: {g_norm:.6f} m/s^2 (expected ~9.81 m/s^2)")
    logging.info(f"Magnitude of omega_ie_body: {omega_norm:.6e} rad/s (expected ~7.29e-5 rad/s)")
    logging.info("==== Measured Vectors in the Body Frame ====")
    logging.info(f"Measured gravity vector (g_body): {g_body} m/s^2")
    logging.info(f"Measured Earth rotation (omega_ie_body): {omega_ie_body} rad/s")
    logging.info("\nNote: These are the same physical vectors as in NED, but expressed in the body frame (sensor axes).")
    logging.info("From accelerometer (assuming static IMU):")
    logging.info("    a_body = -g_body")
    logging.info("From gyroscope:")
    logging.info("    ω_ie,body")
    
    # ================================
    # TASK 3: Solve Wahba’s Problem
    # ================================
    
    
    # ================================
    # TASK 3: Solve Wahba’s Problem
    # ================================
    logging.info("TASK 3: Solve Wahba’s problem (find initial attitude from body to NED)")
    
    # --------------------------------
    # Subtask 3.1: Prepare Vector Pairs for Attitude Determination
    # --------------------------------
    logging.info("Subtask 3.1: Preparing vector pairs for attitude determination.")
    # Case 1: Current implementation vectors
    v1_B = g_body / np.linalg.norm(g_body)  # Normalize gravity in body frame
    v2_B = omega_ie_body / np.linalg.norm(omega_ie_body) if np.linalg.norm(omega_ie_body) > 1e-10 else np.array([1.0, 0.0, 0.0])
    v1_N = g_NED / np.linalg.norm(g_NED)  # Normalize gravity in NED frame
    v2_N = omega_ie_NED / np.linalg.norm(omega_ie_NED)  # Normalize Earth rotation rate
    logging.debug(f"Case 1 - Normalized body gravity: {v1_B}")
    logging.debug(f"Case 1 - Normalized body Earth rate: {v2_B}")
    logging.debug(f"Case 1 - Normalized NED gravity: {v1_N}")
    logging.debug(f"Case 1 - Normalized NED Earth rate: {v2_N}")
    
    # Case 2: Recompute ω_ie,NED using document equation
    omega_E = 7.2921159e-5  # Earth's rotation rate (rad/s)
    omega_ie_NED_doc = omega_E * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    v2_N_doc = omega_ie_NED_doc / np.linalg.norm(omega_ie_NED_doc)  # Normalize for Case 2
    logging.debug(f"Case 2 - Normalized NED Earth rate (document equation): {v2_N_doc}")
    
    # --------------------------------
    # Subtask 3.2: TRIAD Method
    # --------------------------------
    logging.info("Subtask 3.2: Computing rotation matrix using TRIAD method.")
    # Case 1
    t1_body = v1_B
    t2_body_temp = np.cross(v1_B, v2_B)
    if np.linalg.norm(t2_body_temp) < 1e-10:
        logging.warning("Case 1 - Vectors are collinear, TRIAD may fail.")
        t2_body = np.array([1.0, 0.0, 0.0]) if abs(v1_B[0]) < abs(v1_B[1]) else np.array([0.0, 1.0, 0.0])
    else:
        t2_body = t2_body_temp / np.linalg.norm(t2_body_temp)
    t3_body = np.cross(t1_body, t2_body)
    t1_ned = v1_N
    t2_ned_temp = np.cross(v1_N, v2_N)
    if np.linalg.norm(t2_ned_temp) < 1e-10:
        logging.warning("Case 1 - NED vectors are collinear, TRIAD may fail.")
        t2_ned = np.array([1.0, 0.0, 0.0])
    else:
        t2_ned = t2_ned_temp / np.linalg.norm(t2_ned_temp)
    t3_ned = np.cross(t1_ned, t2_ned)
    R_tri = np.column_stack((t1_ned, t2_ned, t3_ned)) @ np.column_stack((t1_body, t2_body, t3_body)).T
    logging.info("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)
    logging.debug("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)
    
    # Case 2
    t2_ned_temp_doc = np.cross(v1_N, v2_N_doc)
    if np.linalg.norm(t2_ned_temp_doc) < 1e-10:
        logging.warning("Case 2 - NED vectors are collinear, TRIAD may fail.")
        t2_ned_doc = np.array([1.0, 0.0, 0.0])
    else:
        t2_ned_doc = t2_ned_temp_doc / np.linalg.norm(t2_ned_temp_doc)
    t3_ned_doc = np.cross(t1_ned, t2_ned_doc)
    R_tri_doc = np.column_stack((t1_ned, t2_ned_doc, t3_ned_doc)) @ np.column_stack((t1_body, t2_body, t3_body)).T
    logging.info("Rotation matrix (TRIAD method, Case 2):\n%s", R_tri_doc)
    logging.debug("Rotation matrix (TRIAD method, Case 2):\n%s", R_tri_doc)
    
    # --------------------------------
    # Subtask 3.3: Davenport’s Q-Method
    # --------------------------------
    logging.info("Subtask 3.3: Computing rotation matrix using Davenport’s Q-Method.")
    w_gravity = 0.9999
    w_omega = 0.0001
    
    # Case 1
    B = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N, v2_B)
    sigma = np.trace(B)
    S = B + B.T
    Z = np.array([B[1, 2] - B[2, 1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]])
    K = np.zeros((4, 4))
    K[0, 0] = sigma
    K[0, 1:] = Z
    K[1:, 0] = Z
    K[1:, 1:] = S - sigma * np.eye(3)
    eigvals, eigvecs = np.linalg.eigh(K)
    q_dav = eigvecs[:, np.argmax(eigvals)]
    if q_dav[0] < 0:
        q_dav = -q_dav
    q_dav = np.array([q_dav[0], -q_dav[1], -q_dav[2], -q_dav[3]])  # Conjugate for body-to-NED
    qw, qx, qy, qz = q_dav
    R_dav = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    logging.info("Rotation matrix (Davenport’s Q-Method, Case 1):\n%s", R_dav)
    logging.info("Davenport quaternion (q_w, q_x, q_y, q_z, Case 1): %s", q_dav)
    logging.debug("Rotation matrix (Davenport’s Q-Method, Case 1):\n%s", R_dav)
    logging.debug("Davenport quaternion (Case 1): %s", q_dav)
    
    # Case 2
    B_doc = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N_doc, v2_B)
    sigma_doc = np.trace(B_doc)
    S_doc = B_doc + B_doc.T
    Z_doc = np.array([B_doc[1, 2] - B_doc[2, 1], B_doc[2, 0] - B_doc[0, 2], B_doc[0, 1] - B_doc[1, 0]])
    K_doc = np.zeros((4, 4))
    K_doc[0, 0] = sigma_doc
    K_doc[0, 1:] = Z_doc
    K_doc[1:, 0] = Z_doc
    K_doc[1:, 1:] = S_doc - sigma_doc * np.eye(3)
    eigvals_doc, eigvecs_doc = np.linalg.eigh(K_doc)
    q_dav_doc = eigvecs_doc[:, np.argmax(eigvals_doc)]
    if q_dav_doc[0] < 0:
        q_dav_doc = -q_dav_doc
    q_dav_doc = np.array([q_dav_doc[0], -q_dav_doc[1], -q_dav_doc[2], -q_dav_doc[3]])
    qw, qx, qy, qz = q_dav_doc
    R_dav_doc = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    logging.info("Rotation matrix (Davenport’s Q-Method, Case 2):\n%s", R_dav_doc)
    logging.info("Davenport quaternion (q_w, q_x, q_y, q_z, Case 2): %s", q_dav_doc)
    logging.debug("Rotation matrix (Davenport’s Q-Method, Case 2):\n%s", R_dav_doc)
    logging.debug("Davenport quaternion (Case 2): %s", q_dav_doc)
    
    # --------------------------------
    # Subtask 3.4: SVD Method
    # --------------------------------
    logging.info("Subtask 3.4: Computing rotation matrix using SVD method.")
    body_vecs = [g_body, omega_ie_body]
    ref_vecs = [g_NED, omega_ie_NED]
    if mag_body is not None and mag_NED is not None:
        body_vecs.append(mag_body)
        ref_vecs.append(mag_NED)
    elif args.use_gnss_heading:
        speed = np.linalg.norm(initial_vel_ned)
        if speed > 0.2:
            body_vecs.append(np.array([1.0, 0.0, 0.0]))
            ref_vecs.append(initial_vel_ned / speed)

    R_svd = svd_alignment(body_vecs, ref_vecs)
    logging.info("Rotation matrix (SVD method):\n%s", R_svd)
    logging.debug("Rotation matrix (SVD method):\n%s", R_svd)
    R_svd_doc = R_svd
    
    # --------------------------------
    # Subtask 3.5: Convert TRIAD and SVD DCMs to Quaternions
    # --------------------------------
    logging.info("Subtask 3.5: Converting TRIAD and SVD DCMs to quaternions.")
    def rot_to_quaternion(R):
        tr = R[0,0] + R[1,1] + R[2,2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
        q = np.array([qw, qx, qy, qz])
        return q / np.linalg.norm(q)
    
    q_tri = rot_to_quaternion(R_tri)
    if q_tri[0] < 0:
        q_tri = -q_tri
    q_svd = rot_to_quaternion(R_svd)
    if q_svd[0] < 0:
        q_svd = -q_svd
    q_tri_doc = rot_to_quaternion(R_tri_doc)
    if q_tri_doc[0] < 0:
        q_tri_doc = -q_tri_doc
    q_svd_doc = rot_to_quaternion(R_svd_doc)
    if q_svd_doc[0] < 0:
        q_svd_doc = -q_svd_doc

    # Combine all methods by averaging rotation matrices
    R_all = average_rotation_matrices([R_tri, R_dav, R_svd])
    q_all = rot_to_quaternion(R_all)
    if q_all[0] < 0:
        q_all = -q_all

    logging.info(f"Quaternion (TRIAD, Case 1): {q_tri}")
    logging.debug(f"Quaternion (TRIAD, Case 1): {q_tri}")
    euler_tri = R.from_matrix(R_tri).as_euler('xyz', degrees=True)
    logging.info(
        f"TRIAD initial attitude (deg): roll={euler_tri[0]:.3f} pitch={euler_tri[1]:.3f} yaw={euler_tri[2]:.3f}"
    )
    with open("triad_init_log.txt", "a") as logf:
        logf.write(
            f"{imu_file}: init_euler_deg={euler_tri}\n"
        )
    logging.info(f"Quaternion (SVD, Case 1): {q_svd}")
    logging.debug(f"Quaternion (SVD, Case 1): {q_svd}")
    logging.info(f"Quaternion (TRIAD, Case 2): {q_tri_doc}")
    logging.debug(f"Quaternion (TRIAD, Case 2): {q_tri_doc}")
    logging.info(f"Quaternion (SVD, Case 2): {q_svd_doc}")
    logging.debug(f"Quaternion (SVD, Case 2): {q_svd_doc}")

    # -- Error metrics for each method ------------------------------------
    logging.info("\nAttitude errors using reference vectors:")

    grav_errors = {}
    omega_errors = {}

    for m, rot_matrix in {
        "TRIAD": R_tri,
        "Davenport": R_dav,
        "SVD": R_svd,
    }.items():
        grav_err_deg, omega_err_deg = compute_wahba_errors(
            rot_matrix, g_body, omega_ie_body, g_NED, omega_ie_NED
        )
        logging.info(f"{m:10s} -> Gravity error (deg): {grav_err_deg:.6f}")
        logging.info(f"{m:10s} -> Earth rate error (deg):  {omega_err_deg:.6f}")
        grav_errors[m] = grav_err_deg
        omega_errors[m] = omega_err_deg

    grav_err_mean = float(np.mean(list(grav_errors.values())))
    grav_err_max  = float(np.max(list(grav_errors.values())))
    omega_err_mean = float(np.mean(list(omega_errors.values())))
    omega_err_max  = float(np.max(list(omega_errors.values())))

    # --------------------------------
    # Subtask 3.6: Validate Attitude Determination and Compare Methods
    # --------------------------------
    logging.info("Subtask 3.6: Validating attitude determination and comparing methods.")
    # -- Composite quaternion and per-method errors ---------------------------
    quats_case1 = {'TRIAD': q_tri, 'Davenport': q_dav, 'SVD': q_svd}
    quats_case2 = {'TRIAD': q_tri_doc, 'Davenport': q_dav_doc, 'SVD': q_svd_doc}

    methods = ["TRIAD", "Davenport", "SVD"]

    def normalise(q):
        return q / np.linalg.norm(q)

    q_all_1 = normalise(sum(quats_case1[m] for m in methods))
    q_all_2 = normalise(sum(quats_case2[m] for m in methods))

    def attitude_errors(q1, q2):
        def quat_to_rot(q):
            w, x, y, z = q
            return np.array([
                [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
                [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
                [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2)]
            ])

        R1 = quat_to_rot(q1)
        R2 = quat_to_rot(q2)
        g_vec = np.array([0.0, 0.0, 1.0])
        w_vec = np.array([1.0, 0.0, 0.0])

        def ang(a, b):
            dot = np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0)
            return np.degrees(np.arccos(dot))

        g_err = ang(R1 @ g_vec, R2 @ g_vec)
        w_err = ang(R1 @ w_vec, R2 @ w_vec)
        return g_err, w_err

    results = {}
    for m in methods:
        g_err, w_err = attitude_errors(quats_case1[m], quats_case2[m])
        results[m] = {"gravity_error": g_err, "earth_rate_error": w_err}

    logging.info("\nDetailed Earth-Rate Errors:")
    for m, o in omega_errors.items():
        logging.info(f"  {m:10s}: {o:.6f}°")

    # Relaxed Earth-rate error check --------------------------------------

    omega_errs = {
        'TRIAD':     omega_errors['TRIAD'],
        'Davenport': omega_errors['Davenport'],
        'SVD':       omega_errors['SVD']
    }
    diff = max(omega_errs.values()) - min(omega_errs.values())
    tol = 1e-5   # allow up to 0.00001° of spread without complaint

    # always print them so you can see the tiny spreads at runtime
    logging.info("\nEarth-rate errors by method:")
    for name, err in omega_errs.items():
        logging.info(f"  {name:10s}: {err:.9f}°")
    logging.info(f"  Δ = {diff:.2e}° (tolerance = {tol:.1e})\n")

    if diff < tol:
        logging.warning(
            "All Earth-rate errors are very close; differences "
            f"are within {tol:.1e}°"
        )

    logging.info("\n==== Method Comparison for Case X001 and Case X001_doc ====")
    logging.info(f"{'Method':10s}  {'Gravity Err (deg)':>18s}  {'Earth-Rate Err (deg)':>22s}")
    for m in ['TRIAD','Davenport','SVD']:
        g = grav_errors[m]
        o = omega_errors[m]
        logging.info(f"{m:10s}  {g:18.4f}  {o:22.4f}")
    
    # --------------------------------
    # Subtask 3.7: Plot Validation Errors and Quaternion Components
    # --------------------------------
    
    
    logging.info("Subtask 3.7: Plotting validation errors and quaternion components.")
    
    methods_plot = methods

    gravity_errors = [results[m]['gravity_error'] for m in methods_plot]
    earth_rate_errors = [results[m]['earth_rate_error'] for m in methods_plot]


    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    x = np.arange(len(methods_plot))
    width = 0.35

    axes[0].bar(x, gravity_errors, width)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(methods_plot)
    axes[0].set_title('Gravity Error')
    axes[0].set_ylabel('Error (degrees)')

    axes[1].bar(x, earth_rate_errors, width)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(methods_plot)
    axes[1].set_title('Earth Rate Error')
    axes[1].set_ylabel('Error (degrees)')
    
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task3_errors_comparison.pdf")
    plt.close()
    logging.info("Error comparison plot saved")
    
    # Collect quaternion data for both cases
    cases = ['Case 1', 'Case 2']
    
    # Plot quaternion components
    fig, ax = plt.subplots(figsize=(12, 6))
    labels = [f'{m} ({c})' for c in cases for m in methods]  # e.g., 'TRIAD (Case 1)', 'TRIAD (Case 2)', etc.
    x = np.arange(len(labels))
    width = 0.15
    components = ['qw', 'qx', 'qy', 'qz']
    
    # Flatten quaternion data for plotting
    all_quats = [quats_case1[m] for m in methods] + [quats_case2[m] for m in methods]
    
    # Plot bars for each quaternion component
    for i, comp in enumerate(components):
        comp_values = [q[i] for q in all_quats]
        ax.bar(x + i*width, comp_values, width, label=comp)
    
    ax.set_xticks(x + 1.5*width)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.set_ylabel('Quaternion Component Value')
    ax.set_title('Quaternion Components for Each Method and Case')
    ax.legend()
    
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task3_quaternions_comparison.pdf")
    plt.close()
    logging.info("Quaternion comparison plot saved")
    
    # --------------------------------
    # Subtask 3.8: Store Rotation Matrices for Later Tasks
    # --------------------------------
    logging.info("Subtask 3.8: Storing rotation matrices for use in later tasks.")
    task3_results = {
        'TRIAD': {'R': R_tri},
        'Davenport': {'R': R_dav},
        'SVD': {'R': R_svd}
    }
    logging.info("Task 3 results stored in memory: %s", list(task3_results.keys()))
        
    # ================================
    # TASK 4: GNSS and IMU Data Integration and Comparison
    # ================================
    
    
    # ================================
    # TASK 4: GNSS and IMU Data Integration and Comparison
    # ================================
    logging.info("TASK 4: GNSS and IMU Data Integration and Comparison")
    
    # --------------------------------
    # Subtask 4.1: Access Rotation Matrices from Task 3
    # --------------------------------
    logging.info("Subtask 4.1: Accessing rotation matrices from Task 3.")
    methods = [method]
    C_B_N_methods = {m: task3_results[m]['R'] for m in methods}
    logging.info("Rotation matrices accessed: %s", list(C_B_N_methods.keys()))
    
    # --------------------------------
    # Subtask 4.3: Load GNSS Data
    # --------------------------------
    logging.info("Subtask 4.3: Loading GNSS data.")
    try:
        gnss_data = pd.read_csv(gnss_file)
    except Exception as e:
        logging.error(f"Failed to load GNSS data file: {e}")
        raise
    
    # --------------------------------
    # Subtask 4.4: Extract Relevant Columns
    # --------------------------------
    logging.info("Subtask 4.4: Extracting relevant columns.")
    time_col = 'Posix_Time'
    pos_cols = ['X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m']
    vel_cols = ['VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps']
    gnss_time = gnss_data[time_col].values
    gnss_pos_ecef = gnss_data[pos_cols].values
    gnss_vel_ecef = gnss_data[vel_cols].values
    logging.info(f"GNSS data shape: {gnss_pos_ecef.shape}")
    
    # --------------------------------
    # Subtask 4.5: Define Reference Point
    # --------------------------------
    logging.info("Subtask 4.5: Defining reference point.")
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = np.array([x_ecef, y_ecef, z_ecef])
    logging.info(f"Reference point: lat={ref_lat:.6f} rad, lon={ref_lon:.6f} rad, r0={ref_r0}")
    
    # --------------------------------
    # Subtask 4.6: Compute Rotation Matrix
    # --------------------------------
    logging.info("Subtask 4.6: Computing ECEF to NED rotation matrix.")
    C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    logging.info("ECEF to NED rotation matrix computed.")
    C_NED_to_ECEF = C_ECEF_to_NED.T
    logging.info("NED to ECEF rotation matrix computed.")
    
    # --------------------------------
    # Subtask 4.7: Convert GNSS Data to NED Frame
    # --------------------------------
    logging.info("Subtask 4.7: Converting GNSS data to NED frame.")
    from utils import ecef_to_ned
    gnss_pos_ned = ecef_to_ned(gnss_pos_ecef, ref_lat, ref_lon, ref_r0)
    gnss_vel_ned = np.array([C_ECEF_to_NED @ v for v in gnss_vel_ecef])
    logging.info("GNSS data transformed to NED frame.")
    
    # --------------------------------
    # Subtask 4.8: Estimate GNSS Acceleration in NED
    # --------------------------------
    logging.info("Subtask 4.8: Estimating GNSS acceleration in NED.")
    gnss_acc_ecef = np.zeros_like(gnss_vel_ecef)
    dt = np.diff(gnss_time, prepend=gnss_time[0])
    gnss_acc_ecef[1:] = (gnss_vel_ecef[1:] - gnss_vel_ecef[:-1]) / dt[1:, np.newaxis]
    gnss_acc_ned = np.array([C_ECEF_to_NED @ a for a in gnss_acc_ecef])
    logging.info("GNSS acceleration estimated in NED frame.")

    acc_biases = {}
    gyro_biases = {}
    
     
    
    # --------------------------------
    # Subtask 4.9: Load IMU Data and Correct for Bias for Each Method
    # --------------------------------
    logging.info("Subtask 4.9: Loading IMU data and correcting for bias for each method.")
    try:
        imu_data = pd.read_csv(imu_file, sep='\s+', header=None)
        dt_ilu = 1.0 / 400.0  # 400 Hz sampling rate, dt = 0.0025 s
        imu_time = np.arange(len(imu_data)) * dt_ilu + gnss_time[0]  # Align with GNSS start time
        
        # Convert velocity increments to acceleration (m/s²)
        # Columns 5,6,7 are velocity increments (m/s) over dt_ilu
        acc_body = imu_data[[5,6,7]].values / dt_ilu  # acc_body = delta_v / dt_ilu
        gyro_body = imu_data[[2,3,4]].values / dt_ilu  # gyro_body = delta_theta / dt_ilu
        acc_body = butter_lowpass_filter(acc_body)
        gyro_body = butter_lowpass_filter(gyro_body)
        
        # Use up to the first 4000 samples (10 s) for bias estimation but
        # fall back to the available amount when running truncated logs.
        N_static = min(4000, len(imu_data))
        if N_static < MIN_STATIC_SAMPLES:
            raise ValueError(
                f"Insufficient static samples for bias estimation; require at least {MIN_STATIC_SAMPLES}."
            )
        
        # Compute static bias for accelerometers and gyroscopes
        static_acc = np.mean(acc_body[:N_static], axis=0)
        static_gyro = np.mean(gyro_body[:N_static], axis=0)
        
    
        # Compute corrected acceleration and gyroscope data for each method
        acc_body_corrected = {}
        gyro_body_corrected = {}
        for m in methods:
            C_N_B = C_B_N_methods[m].T  # NED to Body rotation matrix
            g_body_expected = C_N_B @ g_NED  # Expected gravity in body frame

            # Accelerometer bias: static_acc should equal -g_body_expected (since a_body = f_body - g_body)
            acc_bias = static_acc + g_body_expected  # Bias = measured - expected
            scale = 9.81 / np.linalg.norm(static_acc - acc_bias)

            # Gyroscope bias: static_gyro should equal C_N_B @ omega_ie_NED
            omega_ie_NED = np.array([7.2921159e-5 * np.cos(ref_lat), 0.0, -7.2921159e-5 * np.sin(ref_lat)])
            omega_ie_body_expected = C_N_B @ omega_ie_NED
            gyro_bias = static_gyro - omega_ie_body_expected  # Bias = measured - expected

            # Correct the entire dataset
            acc_body_corrected[m] = scale * (acc_body - acc_bias)
            gyro_body_corrected[m] = gyro_body - gyro_bias
            acc_biases[m] = acc_bias
            gyro_biases[m] = gyro_bias

            logging.info(f"Method {m}: Accelerometer bias: {acc_bias}")
            logging.info(f"Method {m}: Gyroscope bias: {gyro_bias}")
            logging.info(f"Method {m}: Accelerometer scale factor: {scale:.4f}")
            logging.debug(f"Method {m}: Accelerometer bias: {acc_bias}")
            logging.debug(f"Method {m}: Gyroscope bias: {gyro_bias}")
        
        logging.info("IMU data corrected for bias for each method.")
    except Exception as e:
        logging.error(f"Failed to load IMU data or compute corrections: {e}")
        raise
    
    # --------------------------------
    # Subtask 4.10: Set IMU Parameters and Gravity Vector
    # --------------------------------
    logging.info("Subtask 4.10: Setting IMU parameters and gravity vector.")
    g_NED = np.array([0.0, 0.0, 9.81])
    logging.info(f"Gravity vector set: {g_NED}")
    
    # --------------------------------
    # Subtask 4.11: Initialize Output Arrays
    # --------------------------------
    logging.info("Subtask 4.11: Initializing output arrays.")
    # per-method integration results
    pos_integ = {}
    vel_integ = {}
    acc_integ = {}
    
    # --------------------------------
    # Subtask 4.12: Integrate IMU Accelerations for Each Method
    # --------------------------------
    logging.info("Subtask 4.12: Integrating IMU accelerations for each method.")
    for m in methods:
        logging.info(f"Integrating IMU data using {m} method.")
        C_B_N = C_B_N_methods[m]
        pos = np.zeros((len(imu_time), 3))
        vel = np.zeros((len(imu_time), 3))
        acc = np.zeros((len(imu_time), 3))
        for i in range(1, len(imu_time)):
            dt = imu_time[i] - imu_time[i-1]
            f_ned = C_B_N @ acc_body_corrected[m][i]
            a_ned = f_ned + g_NED
            acc[i] = a_ned
            vel[i] = vel[i-1] + a_ned * dt
            pos[i] = pos[i-1] + vel[i] * dt
        pos_integ[m] = pos.copy()
        vel_integ[m] = vel.copy()
        acc_integ[m] = acc.copy()
    logging.info("IMU-derived position, velocity, and acceleration computed for all methods.")
    
    # --------------------------------
    # Subtask 4.13: Validate and Plot Data
    # --------------------------------
    logging.info("Subtask 4.13: Validating and plotting data.")
    t0 = gnss_time[0]
    t_rel_ilu = imu_time - t0
    t_rel_gnss = gnss_time - t0
    # Validate time ranges
    if t_rel_ilu.max() < 1000:
        logging.warning(f"IMU time range too short: {t_rel_ilu.max():.2f} seconds")
    if t_rel_gnss.max() < 1000:
        logging.warning(f"GNSS time range too short: {t_rel_gnss.max():.2f} seconds")
    logging.debug(f"gnss_time range: {gnss_time.min():.2f} to {gnss_time.max():.2f}")
    logging.debug(f"imu_time range: {imu_time.min():.2f} to {imu_time.max():.2f}")
    logging.debug(f"t_rel_gnss range: {t_rel_gnss.min():.2f} to {t_rel_gnss.max():.2f}")
    logging.debug(f"t_rel_ilu range: {t_rel_ilu.min():.2f} to {t_rel_ilu.max():.2f}")

    missing = [m for m in methods if m not in pos_integ]
    if missing:
        logging.warning("Skipping plotting for %s (no data)", missing)
    
    # Comparison plot in NED frame
    fig_comp, axes_comp = plt.subplots(3, 3, figsize=(15, 10))
    directions = ['North', 'East', 'Down']
    colors = COLORS
    for j in range(3):
        # Position comparison
        ax = axes_comp[0, j]
        ax.plot(t_rel_gnss, gnss_pos_ned[:, j], 'k--', label='GNSS Position (direct)')
        for m in methods:
            c = colors.get(m, None)
            ax.plot(t_rel_ilu, pos_integ[m][:, j], color=c, alpha=0.7, label=f'IMU {m} Position (derived)')
        ax.set_title(f'Position {directions[j]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.legend()
        
        # Velocity comparison
        ax = axes_comp[1, j]
        ax.plot(t_rel_gnss, gnss_vel_ned[:, j], 'k--', label='GNSS Velocity (direct)')
        for m in methods:
            c = colors.get(m, None)
            ax.plot(t_rel_ilu, vel_integ[m][:, j], color=c, alpha=0.7, label=f'IMU {m} Velocity (derived)')
        ax.set_title(f'Velocity {directions[j]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        
        # Acceleration comparison
        ax = axes_comp[2, j]
        ax.plot(t_rel_gnss, gnss_acc_ned[:, j], 'k--', label='GNSS Acceleration (derived)')
        for m in methods:
            c = colors.get(m, None)
            ax.plot(t_rel_ilu, acc_integ[m][:, j], color=c, alpha=0.7, label=f'IMU {m} Acceleration (direct)')
        ax.set_title(f'Acceleration {directions[j]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task4_comparison_ned.pdf")
    plt.close()
    logging.info("Comparison plot in NED frame saved")
    
    # Plot 1: Data in mixed frames (GNSS position/velocity in ECEF, IMU acceleration in body)
    logging.info("Plotting data in mixed frames.")
    fig_mixed, axes_mixed = plt.subplots(3, 3, figsize=(15, 10))
    directions_pos = ['X_ECEF', 'Y_ECEF', 'Z_ECEF']
    directions_vel = ['VX_ECEF', 'VY_ECEF', 'VZ_ECEF']
    directions_acc = ['AX_body', 'AY_body', 'AZ_body']
    for i in range(3):
        for j in range(3):
            ax = axes_mixed[i, j]
            if i == 0:  # Position
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], 'k-', label='GNSS Position (ECEF)')
                ax.set_title(f'Position {directions_pos[j]}')
            elif i == 1:  # Velocity
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], 'k-', label='GNSS Velocity (ECEF)')
                ax.set_title(f'Velocity {directions_vel[j]}')
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(t_rel_ilu, acc_body_corrected[m][:, j], color=c, alpha=0.7, label=f'IMU {m} Acceleration (Body)')
                ax.set_title(f'Acceleration {directions_acc[j]}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task4_mixed_frames.pdf")
    plt.close()
    logging.info("Mixed frames plot saved")
    
    # Plot 2: All data in NED frame
    logging.info("Plotting all data in NED frame.")
    fig_ned, axes_ned = plt.subplots(3, 3, figsize=(15, 10))
    directions_ned = ['N', 'E', 'D']
    for i in range(3):
        for j in range(3):
            ax = axes_ned[i, j]
            if i == 0:  # Position
                ax.plot(t_rel_gnss, gnss_pos_ned[:, j], 'k-', label='GNSS Position (NED)')
                ax.set_title(f'Position {directions_ned[j]}')
            elif i == 1:  # Velocity
                ax.plot(t_rel_gnss, gnss_vel_ned[:, j], 'k-', label='GNSS Velocity (NED)')
                ax.set_title(f'Velocity V{directions_ned[j]}')
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                    ax.plot(t_rel_ilu, f_ned[j], color=c, alpha=0.7, label=f'IMU {m} Specific Force (NED)')
                ax.set_title(f'Specific Force f{directions_ned[j]}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task4_all_ned.pdf")
    plt.close()
    logging.info("All data in NED frame plot saved")
    
    # Plot 3: All data in ECEF frame
    logging.info("Plotting all data in ECEF frame.")
    fig_ecef, axes_ecef = plt.subplots(3, 3, figsize=(15, 10))
    directions_ecef = ['X', 'Y', 'Z']
    for i in range(3):
        for j in range(3):
            ax = axes_ecef[i, j]
            if i == 0:  # Position
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], 'k-', label='GNSS Position (ECEF)')
                ax.set_title(f'Position {directions_ecef[j]}_ECEF')
            elif i == 1:  # Velocity
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], 'k-', label='GNSS Velocity (ECEF)')
                ax.set_title(f'Velocity V{directions_ecef[j]}_ECEF')
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                    f_ecef = C_NED_to_ECEF @ f_ned
                    ax.plot(t_rel_ilu, f_ecef[j], color=c, alpha=0.7, label=f'IMU {m} Specific Force (ECEF)')
                ax.set_title(f'Specific Force f{directions_ecef[j]}_ECEF')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task4_all_ecef.pdf")
    plt.close()
    logging.info("All data in ECEF frame plot saved")
    
    # Plot 4: All data in body frame
    logging.info("Plotting all data in body frame.")
    fig_body, axes_body = plt.subplots(3, 3, figsize=(15, 10))
    directions_body = ['X', 'Y', 'Z']
    for i in range(3):
        for j in range(3):
            ax = axes_body[i, j]
            if i == 0:  # Position
                r_body = (C_N_B @ gnss_pos_ned.T).T
                ax.plot(t_rel_gnss, r_body[:, j], 'k-', label='Position in body frame')
                ax.set_title(f'Position r{directions_body[j]}_body')
            elif i == 1:  # Velocity
                vel_body = (C_N_B @ gnss_vel_ned.T).T
                ax.plot(t_rel_gnss, vel_body[:, j], 'k-', label='Velocity in body frame')
                ax.set_title(f'Velocity v{directions_body[j]}_body')
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(t_rel_ilu, acc_body_corrected[m][:, j], color=c, alpha=0.7, label=f'IMU {m} Acceleration (Body)')
                ax.set_title(f'Acceleration A{directions_body[j]}_body')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task4_all_body.pdf")
    plt.close()
    logging.info("All data in body frame plot saved")
    
    
    
    
    # ================================
    # Task 5: Sensor Fusion with Kalman Filter
    # ================================
    logging.info("Task 5: Sensor Fusion with Kalman Filter")
    
    # --------------------------------
    # Subtask 5.1: Configure Logging
    # --------------------------------
    logging.info("Subtask 5.1: Configuring logging.")
    
    # --------------------------------
    # Subtask 5.2: Rotation Matrix - ECEF to NED
    # --------------------------------
    
    # --------------------------------
    # Subtask 5.3: Load GNSS and IMU Data
    # --------------------------------
    logging.info("Subtask 5.3: Loading GNSS and IMU data.")
    try:
        gnss_data = pd.read_csv(gnss_file)
        imu_data = pd.read_csv(imu_file, sep='\s+', header=None)
    except FileNotFoundError as e:
        missing = 'GNSS' if 'csv' in str(e) else 'IMU'
        logging.error(f"{missing} file not found: {e.filename}")
        raise
    except Exception as e:
        logging.error(f"Failed to load data: {e}")
        raise
    
    # Extract GNSS fields
    time_col = 'Posix_Time'
    pos_cols = ['X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m']
    vel_cols = ['VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps']
    gnss_time = gnss_data[time_col].values
    gnss_pos_ecef = gnss_data[pos_cols].values
    gnss_vel_ecef = gnss_data[vel_cols].values
    
    # Reference position for NED
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = np.array([x_ecef, y_ecef, z_ecef])
    C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon)

    # Convert GNSS to NED
    from utils import ecef_to_ned
    gnss_pos_ned = ecef_to_ned(gnss_pos_ecef, ref_lat, ref_lon, ref_r0)
    gnss_vel_ned = np.array([C_ECEF_to_NED @ v for v in gnss_vel_ecef])
    
    # Compute GNSS acceleration
    gnss_acc_ecef = np.zeros_like(gnss_vel_ecef)
    dt = np.diff(gnss_time, prepend=gnss_time[0])
    gnss_acc_ecef[1:] = (gnss_vel_ecef[1:] - gnss_vel_ecef[:-1]) / dt[1:, np.newaxis]
    gnss_acc_ned = np.array([C_ECEF_to_NED @ a for a in gnss_acc_ecef])
    
    # Load IMU data
    dt_ilu = 1.0 / 400.0
    imu_time = np.arange(len(imu_data)) * dt_ilu + gnss_time[0]
    acc_body = imu_data[[5,6,7]].values / dt_ilu
    acc_body = butter_lowpass_filter(acc_body)
    # Use at most 4000 samples but allow shorter sequences when running the
    # trimmed datasets used in unit tests.
    N_static = min(4000, len(imu_data))
    if N_static < MIN_STATIC_SAMPLES:
        raise ValueError(
            f"Insufficient static samples; require at least {MIN_STATIC_SAMPLES}."
        )
    static_acc = np.mean(acc_body[:N_static], axis=0)
    
    
    
    # Compute corrected acceleration for each method
    acc_body_corrected = {}
    for m in methods:
        C_N_B = C_B_N_methods[m].T
        g_body_expected = C_N_B @ g_NED
        bias = static_acc + g_body_expected
        scale = 9.81 / np.linalg.norm(static_acc - bias)
        acc_body_corrected[m] = scale * (acc_body - bias)
        logging.info(f"Method {m}: Bias computed: {bias}")
        logging.info(f"Method {m}: Scale factor: {scale:.4f}")
    
    # --------------------------------
    # Subtask 5.4: Integrate IMU Data for Each Method
    # --------------------------------
    logging.info("Subtask 5.4: Integrating IMU data for each method.")
    imu_pos = {m: np.zeros((len(imu_time), 3)) for m in methods}
    imu_vel = {m: np.zeros((len(imu_time), 3)) for m in methods}
    imu_acc = {m: np.zeros((len(imu_time), 3)) for m in methods}
    for m in methods:
        C_B_N = C_B_N_methods[m]
        imu_pos[m][0] = gnss_pos_ned[0]
        imu_vel[m][0] = gnss_vel_ned[0]
        for i in range(1, len(imu_time)):
            dt = imu_time[i] - imu_time[i-1]
            f_ned = C_B_N @ acc_body_corrected[m][i]
            a_ned = f_ned + g_NED
            imu_acc[m][i] = a_ned
            imu_vel[m][i] = imu_vel[m][i-1] + 0.5 * (imu_acc[m][i] + imu_acc[m][i-1]) * dt
            imu_pos[m][i] = imu_pos[m][i-1] + 0.5 * (imu_vel[m][i] + imu_vel[m][i-1]) * dt
        logging.info(f"Method {m}: IMU data integrated.")
    
    # ZUPT handled dynamically during Kalman filtering
    
    # --------------------------------
    # Subtask 5.6: Kalman Filter for Sensor Fusion for Each Method
    # --------------------------------
    logging.info("Subtask 5.6: Running Kalman Filter for sensor fusion for each method.")
    fused_pos = {m: np.zeros_like(imu_pos[m]) for m in methods}
    fused_vel = {m: np.zeros_like(imu_vel[m]) for m in methods}
    fused_acc = {m: np.zeros_like(imu_acc[m]) for m in methods}
    # Interpolate GNSS data to IMU time once
    gnss_pos_ned_interp = np.zeros((len(imu_time), 3))
    gnss_vel_ned_interp = np.zeros((len(imu_time), 3))
    for j in range(3):
        gnss_pos_ned_interp[:, j] = np.interp(imu_time, gnss_time, gnss_pos_ned[:, j])
        gnss_vel_ned_interp[:, j] = np.interp(imu_time, gnss_time, gnss_vel_ned[:, j])

    innov_pos_all = {}
    innov_vel_all = {}
    attitude_q_all = {}
    euler_all = {}
    res_pos_all = {}
    res_vel_all = {}
    time_res_all = {}
    P_hist_all = {}
    zupt_counts = {}
    zupt_events_all = {}

    for m in methods:
        kf = KalmanFilter(dim_x=9, dim_z=6)
        kf.x = np.hstack((imu_pos[m][0], imu_vel[m][0], imu_acc[m][0]))
        kf.F = np.eye(9)
        kf.H = np.hstack((np.eye(6), np.zeros((6,3))))
        kf.P *= 1.0
        kf.R = np.eye(6) * 0.1
        kf.Q = np.eye(9) * 0.01
        fused_pos[m][0] = imu_pos[m][0]
        fused_vel[m][0] = imu_vel[m][0]
        fused_acc[m][0] = imu_acc[m][0]

        # ---------- logging containers ----------
        innov_pos = []      # GNSS - predicted position
        innov_vel = []      # GNSS - predicted velocity
        attitude_q = []     # quaternion history
        res_pos_list, res_vel_list, time_res = [], [], []
        euler_list = []

        win = 80
        acc_win = []
        gyro_win = []
        zupt_count = 0
        zupt_events = []
        P_hist = [kf.P.copy()]

        # attitude initialisation for logging
        orientations = np.zeros((len(imu_time), 4))
        initial_quats = {'TRIAD': q_tri, 'Davenport': q_dav, 'SVD': q_svd}
        orientations[0] = initial_quats[m]
        attitude_q.append(orientations[0])
        q_cur = orientations[0]
        roll, pitch, yaw = quat2euler(q_cur)
        euler_list.append([roll, pitch, yaw])
        

        
        # Run Kalman Filter
        for i in range(1, len(imu_time)):
            dt = imu_time[i] - imu_time[i-1]

            # propagate quaternion using gyro measurement
            dq = quat_from_rate(gyro_body_corrected[m][i], dt)
            q_cur = quat_multiply(q_cur, dq)
            q_cur /= np.linalg.norm(q_cur)
            orientations[i] = q_cur

            # Prediction step
            kf.F[0:3,3:6] = np.eye(3) * dt
            kf.F[3:6,6:9] = np.eye(3) * dt
            kf.predict()

            # ---------- save attitude BEFORE measurement update ----------
            attitude_q.append(q_cur)

            # ---------- compute and log the innovations BEFORE update ----------
            pred = kf.H @ kf.x
            innov = np.hstack((gnss_pos_ned_interp[i], gnss_vel_ned_interp[i])) - pred
            innov_pos.append(innov[0:3])
            innov_vel.append(innov[3:6])
            res_pos_list.append(innov[0:3])
            res_vel_list.append(innov[3:6])
            time_res.append(imu_time[i])
            roll, pitch, yaw = quat2euler(q_cur)
            euler_list.append([roll, pitch, yaw])

            # Update step
            z = np.hstack((gnss_pos_ned_interp[i], gnss_vel_ned_interp[i]))
            kf.update(z)

            # --- ZUPT check with rolling variance ---
            acc_win.append(acc_body_corrected[m][i])
            gyro_win.append(gyro_body_corrected[m][i])
            if len(acc_win) > win:
                acc_win.pop(0)
                gyro_win.pop(0)
            if len(acc_win) == win and is_static(np.array(acc_win), np.array(gyro_win)):
                H_z = np.zeros((3, 9))
                H_z[:, 3:6] = np.eye(3)
                R_z = np.eye(3) * 1e-4
                pred_v = H_z @ kf.x
                S = H_z @ kf.P @ H_z.T + R_z
                K = kf.P @ H_z.T @ np.linalg.inv(S)
                kf.x += K @ (-pred_v)
                kf.P = (np.eye(9) - K @ H_z) @ kf.P
                zupt_count += 1
                zupt_events.append((i - win + 1, i))
                # This log was previously INFO but produced excessive output
                # during long runs. Use DEBUG level so it only appears when
                # explicitly requested.
                logging.debug(
                    f"ZUPT applied at {imu_time[i]:.2f}s (window {i-win+1}-{i})"
                )

            fused_pos[m][i] = kf.x[0:3]
            fused_vel[m][i] = kf.x[3:6]
            fused_acc[m][i] = imu_acc[m][i]  # Use integrated acceleration
            P_hist.append(kf.P.copy())

        logging.info(f"Method {m}: Kalman Filter completed. ZUPTcnt={zupt_count}")
        with open("triad_init_log.txt", "a") as logf:
            for s, e in zupt_events:
                logf.write(f"{imu_file}: ZUPT {s}-{e}\n")
        zupt_counts[m] = zupt_count
        zupt_events_all[m] = list(zupt_events)

        # stack log lists
        innov_pos = np.vstack(innov_pos)
        innov_vel = np.vstack(innov_vel)
        attitude_q = np.vstack(attitude_q)
        res_pos = np.vstack(res_pos_list)
        res_vel = np.vstack(res_vel_list)
        euler = np.vstack(euler_list)
        time_res_arr = np.array(time_res)

        innov_pos_all[m] = innov_pos
        innov_vel_all[m] = innov_vel
        attitude_q_all[m] = attitude_q
        euler_all[m] = euler
        res_pos_all[m] = res_pos
        res_vel_all[m] = res_vel
        time_res_all[m] = time_res_arr
        P_hist_all[m] = np.stack(P_hist)
    
    # Compute residuals for the selected method
    residual_pos = res_pos_all[method]
    residual_vel = res_vel_all[method]
    time_residuals = time_res_all[method]

    attitude_angles = np.rad2deg(euler_all[method])
    
    # --------------------------------
    # Subtask 5.7: Handle Event at 5000s (if needed)
    # --------------------------------
    logging.info("Subtask 5.7: No event handling needed as time < 5000s.")
    
    # --------------------------------
    # Subtask 5.8: Plotting Results for All Methods
    # --------------------------------
    
    
    
    # Configure logging if not already done
    
    # Define methods and colors
    methods = [method]
    colors = COLORS
    directions = ['North', 'East', 'Down']
    
    # Interpolate GNSS acceleration to IMU time (done once for all plots)
    gnss_acc_ned_interp = np.zeros((len(imu_time), 3))
    for j in range(3):
        gnss_acc_ned_interp[:, j] = np.interp(imu_time, gnss_time, gnss_acc_ned[:, j])
        logging.info(
            f"Interpolated GNSS acceleration for {directions[j]} direction: "
            f"First value = {gnss_acc_ned_interp[0, j]:.4f}, "
            f"Last value = {gnss_acc_ned_interp[-1, j]:.4f}"
        )
        logging.debug(
            f"# Interpolated GNSS acceleration {directions[j]}: "
            f"First = {gnss_acc_ned_interp[0, j]:.4f}, Last = {gnss_acc_ned_interp[-1, j]:.4f}"
        )
    
    
    # Subtask 5.8.2: Plotting Results for selected method
    logging.info(f"Subtask 5.8.2: Plotting results for {method}.")
    logging.debug(f"# Subtask 5.8.2: Starting to plot results for {method}.")
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    
    # Davenport - Position
    for j in range(3):
        ax = axes[0, j]
        ax.plot(imu_time, gnss_pos_ned_interp[:, j], 'k-', label='GNSS')
        c = colors.get(method, None)
        ax.plot(imu_time, fused_pos[method][:, j], c, alpha=0.7, label=f'Fused {method}')
        ax.set_title(f'Position {directions[j]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.legend()
        logging.info(
            f"Subtask 5.8.2: Plotted {method} position {directions[j]}: "
            f"First = {fused_pos[method][0, j]:.4f}, Last = {fused_pos[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} position {directions[j]}: "
            f"First = {fused_pos[method][0, j]:.4f}, Last = {fused_pos[method][-1, j]:.4f}"
        )
    
    # Davenport - Velocity
    for j in range(3):
        ax = axes[1, j]
        ax.plot(imu_time, gnss_vel_ned_interp[:, j], 'k-', label='GNSS')
        c = colors.get(method, None)
        ax.plot(imu_time, fused_vel[method][:, j], c, alpha=0.7, label=f'Fused {method}')
        ax.set_title(f'Velocity {directions[j]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        logging.info(
            f"Subtask 5.8.2: Plotted {method} velocity {directions[j]}: "
            f"First = {fused_vel[method][0, j]:.4f}, Last = {fused_vel[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} velocity {directions[j]}: "
            f"First = {fused_vel[method][0, j]:.4f}, Last = {fused_vel[method][-1, j]:.4f}"
        )
    
    # Davenport - Acceleration
    for j in range(3):
        ax = axes[2, j]
        ax.plot(imu_time, gnss_acc_ned_interp[:, j], 'k-', label='GNSS')
        c = colors.get(method, None)
        ax.plot(imu_time, fused_acc[method][:, j], c, alpha=0.7, label=f'Fused {method}')
        ax.set_title(f'Acceleration {directions[j]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        ax.legend()
        logging.info(
            f"Subtask 5.8.2: Plotted {method} acceleration {directions[j]}: "
            f"First = {fused_acc[method][0, j]:.4f}, Last = {fused_acc[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} acceleration {directions[j]}: "
            f"First = {fused_acc[method][0, j]:.4f}, Last = {fused_acc[method][-1, j]:.4f}"
        )
    
    plt.tight_layout()
    out_pdf = f"results/{tag}_task5_results_{method}.pdf"
    if not args.no_plots:
        save_plot(fig, out_pdf, f"Kalman Filter Results — {tag}")
    logging.info(f"Subtask 5.8.2: {method} plot saved as '{out_pdf}'")
    logging.debug(f"# Subtask 5.8.2: {method} plotting completed. Saved as '{out_pdf}'.")

    # ----- Additional reference frame plots -----
    logging.info("Plotting all data in NED frame.")
    fig_ned_all, ax_ned_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_ned = ['N', 'E', 'D']
    c = colors.get(method, None)
    for i in range(3):
        for j in range(3):
            ax = ax_ned_all[i, j]
            if i == 0:
                ax.plot(t_rel_gnss, gnss_pos_ned[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, imu_pos[method][:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, fused_pos[method][:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Position {dirs_ned[j]}')
            elif i == 1:
                ax.plot(t_rel_gnss, gnss_vel_ned[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, imu_vel[method][:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, fused_vel[method][:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Velocity V{dirs_ned[j]}')
            else:
                ax.plot(t_rel_gnss, gnss_acc_ned[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, imu_acc[method][:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, fused_acc[method][:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Acceleration {dirs_ned[j]}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task5_compare_ned.pdf")
    plt.close()
    logging.info("All data in NED frame plot saved")

    logging.info("Plotting all data in ECEF frame.")
    fig_ecef_all, ax_ecef_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_ecef = ['X', 'Y', 'Z']
    pos_ecef_fused = np.array([C_NED_to_ECEF @ p + ref_r0 for p in fused_pos[method]])
    vel_ecef_fused = (C_NED_to_ECEF @ fused_vel[method].T).T
    acc_ecef_fused = (C_NED_to_ECEF @ fused_acc[method].T).T
    pos_ecef_imu = np.array([C_NED_to_ECEF @ p + ref_r0 for p in imu_pos[method]])
    vel_ecef_imu = (C_NED_to_ECEF @ imu_vel[method].T).T
    acc_ecef_imu = (C_NED_to_ECEF @ imu_acc[method].T).T
    for i in range(3):
        for j in range(3):
            ax = ax_ecef_all[i, j]
            if i == 0:
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, pos_ecef_imu[:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, pos_ecef_fused[:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Position {dirs_ecef[j]}_ECEF')
            elif i == 1:
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, vel_ecef_imu[:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, vel_ecef_fused[:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Velocity V{dirs_ecef[j]}_ECEF')
            else:
                ax.plot(t_rel_gnss, gnss_acc_ecef[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, acc_ecef_imu[:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, acc_ecef_fused[:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Acceleration {dirs_ecef[j]}_ECEF')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task5_compare_ecef.pdf")
    plt.close()
    logging.info("All data in ECEF frame plot saved")

    logging.info("Plotting all data in body frame.")
    fig_body_all, ax_body_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_body = ['X', 'Y', 'Z']
    C_N_B = C_B_N_methods[method].T
    pos_body_fused = (C_N_B @ fused_pos[method].T).T
    vel_body_fused = (C_N_B @ fused_vel[method].T).T
    acc_body_fused = (C_N_B @ fused_acc[method].T).T
    pos_body_imu = (C_N_B @ imu_pos[method].T).T
    vel_body_imu = (C_N_B @ imu_vel[method].T).T
    acc_body_imu = (C_N_B @ imu_acc[method].T).T
    gnss_pos_body = (C_N_B @ gnss_pos_ned.T).T
    gnss_vel_body = (C_N_B @ gnss_vel_ned.T).T
    gnss_acc_body = (C_N_B @ gnss_acc_ned.T).T
    for i in range(3):
        for j in range(3):
            ax = ax_body_all[i, j]
            if i == 0:
                ax.plot(t_rel_gnss, gnss_pos_body[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, pos_body_imu[:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, pos_body_fused[:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Position r{dirs_body[j]}_body')
            elif i == 1:
                ax.plot(t_rel_gnss, gnss_vel_body[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, vel_body_imu[:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, vel_body_fused[:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Velocity v{dirs_body[j]}_body')
            else:
                ax.plot(t_rel_gnss, gnss_acc_body[:, j], 'k-', label='GNSS')
                ax.plot(t_rel_ilu, acc_body_imu[:, j], '--', color='gray',
                        label='IMU (no fusion)')
                ax.plot(t_rel_ilu, acc_body_fused[:, j], c, alpha=0.7,
                        label='Fused (KF)')
                ax.set_title(f'Acceleration A{dirs_body[j]}_body')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value')
            ax.legend()
    plt.tight_layout()
    if not args.no_plots:
        plt.savefig(f"results/{tag}_task5_compare_body.pdf")
    plt.close()
    logging.info("All data in body frame plot saved")

    # Plot pre-fit innovations
    fig_innov, ax_innov = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
    labels = ['North', 'East', 'Down']
    innov_pos = innov_pos_all[method]
    innov_vel = innov_vel_all[method]
    for i in range(3):
        ax_innov[i].plot(innov_pos[:, i], label='Position')
        ax_innov[i].plot(innov_vel[:, i], label='Velocity', linestyle='--')
        ax_innov[i].set_ylabel(f'{labels[i]} residual')
        ax_innov[i].grid(True)
    ax_innov[0].legend(loc='upper right')
    ax_innov[-1].set_xlabel('GNSS update index')
    fig_innov.suptitle('Pre-fit Innovations (GNSS – prediction)')
    fig_innov.tight_layout()
    innov_pdf = f"results/{tag}_{method.lower()}_innovations.pdf"
    if not args.no_plots:
        save_plot(fig_innov, innov_pdf, 'Pre-fit Innovations')
    
    # Plot residuals and attitude using helper functions
    if not args.no_plots:
        res = compute_residuals(gnss_time, gnss_pos_ned, imu_time, fused_pos[method])
        plot_residuals(gnss_time, res,
                       f"results/residuals_{tag}_{method}.pdf")
        plot_attitude(imu_time, attitude_q_all[method],
                      f"results/attitude_angles_{tag}_{method}.pdf")
    
    # Create plot summary
    summary = {
        f'{tag}_location_map.pdf': 'Initial location on Earth map',
        f'{tag}_task3_errors_comparison.pdf': 'Attitude initialization error comparison',
        f'{tag}_task3_quaternions_comparison.pdf': 'Quaternion components for initialization',
        f'{tag}_task4_comparison_ned.pdf': 'GNSS vs IMU data in NED frame',
        f'{tag}_task4_mixed_frames.pdf': 'GNSS/IMU data in mixed frames',
        f'{tag}_task4_all_ned.pdf': 'Integrated data in NED frame',
        f'{tag}_task4_all_ecef.pdf': 'Integrated data in ECEF frame',
        f'{tag}_task4_all_body.pdf': 'Integrated data in body frame',
        f'{tag}_task5_results_{method}.pdf': f'Kalman filter results using {method}',
        f'{tag}_task5_compare_ned.pdf': 'GNSS/IMU/KF comparison in NED frame',
        f'{tag}_task5_compare_ecef.pdf': 'GNSS/IMU/KF comparison in ECEF frame',
        f'{tag}_task5_compare_body.pdf': 'GNSS/IMU/KF comparison in body frame',
        f'{tag}_{method.lower()}_residuals.pdf': 'Position and velocity residuals',
        f'{tag}_{method.lower()}_innovations.pdf': 'Pre-fit innovations',
        f'{tag}_{method.lower()}_attitude_angles.pdf': 'Attitude angles over time'
    }
    summary_path = os.path.join("results", f"{tag}_plot_summary.md")
    with open(summary_path, 'w') as f:
        for name, desc in summary.items():
            f.write(f'- **{name}**: {desc}\n')

    rmse_pos = np.sqrt(np.mean(np.sum((gnss_pos_ned_interp - fused_pos[method])**2, axis=1)))
    final_pos = np.linalg.norm(gnss_pos_ned_interp[-1] - fused_pos[method][-1])

    # --- Additional residual metrics ---------------------------------------
    pos_interp = np.vstack([
        np.interp(gnss_time, imu_time, fused_pos[method][:, i])
        for i in range(3)
    ]).T
    vel_interp = np.vstack([
        np.interp(gnss_time, imu_time, fused_vel[method][:, i])
        for i in range(3)
    ]).T
    resid_pos = pos_interp - gnss_pos_ned
    resid_vel = vel_interp - gnss_vel_ned

    rms_resid_pos = np.sqrt(np.mean(resid_pos ** 2))
    rms_resid_vel = np.sqrt(np.mean(resid_vel ** 2))
    max_resid_pos = np.max(np.linalg.norm(resid_pos, axis=1))
    max_resid_vel = np.max(np.linalg.norm(resid_vel, axis=1))

    accel_bias = acc_biases.get(method, np.zeros(3))
    gyro_bias = gyro_biases.get(method, np.zeros(3))

    # --- Attitude angles ----------------------------------------------------
    euler = R.from_quat(attitude_q_all[method]).as_euler('xyz', degrees=True)
    if not args.no_plots:
        plt.figure()
        plt.plot(imu_time, euler[:, 0], label='Roll')
        plt.plot(imu_time, euler[:, 1], label='Pitch')
        plt.plot(imu_time, euler[:, 2], label='Yaw')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (deg)')
        plt.legend()
        plt.title(f'{tag} Attitude Angles')
        plt.savefig(f'results/{tag}_{method}_attitude_angles.png')
        plt.close()

    np.savez(
        f"results/{tag}_kf_output.npz",
        summary=dict(
            rmse_pos=rmse_pos,
            final_pos=final_pos,
            grav_err_mean=grav_err_mean,
            grav_err_max=grav_err_max,
            earth_rate_err_mean=omega_err_mean,
            earth_rate_err_max=omega_err_max,
        ),
        innov_pos=innov_pos_all[method],
        innov_vel=innov_vel_all[method],
        fused_pos=fused_pos[method],
        fused_vel=fused_vel[method],
        euler=euler_all[method],
        residual_pos=res_pos_all[method],
        residual_vel=res_vel_all[method],
        time_residuals=time_res_all[method],
        attitude_q=attitude_q_all[method],
        P_hist=P_hist_all[method],
    )

    # Also export results as MATLAB-compatible .mat for post-processing
    from scipy.io import savemat
    savemat(
        f"results/{tag}_kf_output.mat",
        {
            "rmse_pos": np.array([rmse_pos]),
            "final_pos": np.array([final_pos]),
            "grav_err_mean": np.array([grav_err_mean]),
            "grav_err_max": np.array([grav_err_max]),
            "earth_rate_err_mean": np.array([omega_err_mean]),
            "earth_rate_err_max": np.array([omega_err_max]),
            "innov_pos": innov_pos_all[method],
            "innov_vel": innov_vel_all[method],
            "fused_pos": fused_pos[method],
            "fused_vel": fused_vel[method],
            "euler": euler_all[method],
            "residual_pos": res_pos_all[method],
            "residual_vel": res_vel_all[method],
            "time_residuals": time_res_all[method],
            "attitude_q": attitude_q_all[method],
            "P_hist": P_hist_all[method],
        },
    )

    # --- Persist for cross-dataset comparison ------------------------------
    import pickle, gzip
    pack = {
        "method"   : method,
        "dataset"  : summary_tag.split("_")[1],
        "t"        : t_rel_ilu,
        "pos_ned"  : fused_pos[method],
        "vel_ned"  : fused_vel[method],
        "pos_gnss" : gnss_pos_ned,
        "vel_gnss" : gnss_vel_ned,
    }
    fname = Path("results") / f"{summary_tag}_{method}_compare.pkl.gz"
    with gzip.open(fname, "wb") as f:
        pickle.dump(pack, f)

    # --- Final plots -------------------------------------------------------
    if not args.no_plots:
        dataset_id = imu_stem.split("_")[1]
        zupt_mask = np.zeros(len(imu_time), dtype=bool)
        for s, e in zupt_events_all.get(method, []):
            s = max(0, s)
            e = min(len(imu_time) - 1, e)
            zupt_mask[s:e+1] = True

        save_zupt_variance(
            acc_body_corrected[method],
            zupt_mask,
            dt_ilu,
            dataset_id,
            threshold=0.01,
        )

        euler_deg = np.rad2deg(euler_all[method])
        save_euler_angles(imu_time, euler_deg, dataset_id)

        pos_f = np.vstack([
            np.interp(gnss_time, imu_time, fused_pos[method][:, i])
            for i in range(3)
        ]).T
        vel_f = np.vstack([
            np.interp(gnss_time, imu_time, fused_vel[method][:, i])
            for i in range(3)
        ]).T
        save_residual_plots(
            gnss_time,
            pos_f,
            gnss_pos_ned,
            vel_f,
            gnss_vel_ned,
            dataset_id,
        )

        save_attitude_over_time(imu_time, euler_deg, dataset_id)

    logging.info(
        f"[SUMMARY] method={method:<9} imu={os.path.basename(imu_file)} gnss={os.path.basename(gnss_file)} "
        f"rmse_pos={rmse_pos:7.2f}m final_pos={final_pos:7.2f}m "
        f"rms_resid_pos={rms_resid_pos:7.2f}m max_resid_pos={max_resid_pos:7.2f}m "
        f"rms_resid_vel={rms_resid_vel:7.2f}m max_resid_vel={max_resid_vel:7.2f}m "
        f"accel_bias={np.linalg.norm(accel_bias):.4f} gyro_bias={np.linalg.norm(gyro_bias):.4f} "
        f"ZUPT_count={zupt_counts.get(method,0)} "
        f"GravErrMean_deg={grav_err_mean:.6f} GravErrMax_deg={grav_err_max:.6f} "
        f"EarthRateErrMean_deg={omega_err_mean:.6f} EarthRateErrMax_deg={omega_err_max:.6f}"
    )
    

if __name__ == "__main__":
    main()
