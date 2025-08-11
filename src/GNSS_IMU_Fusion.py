"""Run the GNSS/IMU fusion pipeline for a single dataset pair.

This script mirrors the MATLAB ``GNSS_IMU_Fusion.m`` workflow and provides
the same command-line interface.  It performs attitude initialisation and
extended Kalman filtering given IMU and GNSS logs.

The accelerometer scale factor estimated in Task 4 is applied once when
correcting raw accelerometer measurements.  If no prior estimate exists a
neutral factor of ``1.0`` is used.

Usage
-----
python src/GNSS_IMU_Fusion.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv

All results are written to ``results/``.
"""

import argparse
import logging
import sys
import os
import io
from pathlib import Path

if __package__ is None:
    # When executed directly, ensure the project root (the parent of this file)
    # is on ``sys.path``.  This allows importing the ``src`` package just like
    # the test suite does.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "src"

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from filterpy.kalman import KalmanFilter

from scripts.plot_utils import save_plot, plot_attitude
from utils import (
    is_static,
    compute_C_ECEF_to_NED,
    ecef_to_geodetic,
    interpolate_series,
    zero_base_time,
)
from constants import GRAVITY, EARTH_RATE
from .compute_biases import compute_biases
from scripts.validate_filter import compute_residuals, plot_residuals
from scipy.spatial.transform import Rotation as R
from python.utils.save_plot_all import save_plot_all
from .gnss_imu_fusion.init_vectors import (
    average_rotation_matrices,
    svd_alignment,
    triad_svd,
    butter_lowpass_filter,
    compute_wahba_errors,
)
from .gnss_imu_fusion.plots import (
    save_zupt_variance,
    save_euler_angles,
    save_residual_plots,
    save_attitude_over_time,
    save_velocity_profile,
    plot_all_methods,
)
from .gnss_imu_fusion.init import compute_reference_vectors, measure_body_vectors
from .gnss_imu_fusion.integration import integrate_trajectory
from .gnss_imu_fusion.kalman_filter import (
    quat_multiply,
    quat_from_rate,
    quat2euler,
)

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
utf8_stdout = io.TextIOWrapper(
    sys.stdout.buffer,
    encoding="utf-8",
    errors="replace",
)
handler = logging.StreamHandler(utf8_stdout)
logging.basicConfig(level=logging.INFO, format="%(message)s", handlers=[handler])

# Minimum number of samples required from a static interval for bias estimation
MIN_STATIC_SAMPLES = 500


def check_files(imu_file: str, gnss_file: str) -> tuple[str, str]:
    """Return validated dataset paths."""
    imu_path = Path(imu_file)
    gnss_path = Path(gnss_file)
    return str(imu_path), str(gnss_path)


def main():
    os.makedirs("results", exist_ok=True)
    logging.info("Ensured 'results/' directory exists.")
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu-file", required=True)
    parser.add_argument("--gnss-file", required=True)
    # Using TRIAD initialization for all tests based on its consistent runtime and accuracy.
    parser.add_argument(
        "--method",
        default="TRIAD",
        choices=["TRIAD", "Davenport", "SVD"],
    )
    parser.add_argument("--mag-file", help="CSV file with magnetometer data")
    parser.add_argument(
        "--truth-file",
        help="Optional ground truth trajectory in STATE_X001.txt format",
    )
    parser.add_argument(
        "--use-gnss-heading",
        action="store_true",
        help="Use initial GNSS velocity for yaw if no magnetometer",
    )
    parser.add_argument("--accel-noise", type=float, default=0.1)
    parser.add_argument("--accel-bias-noise", type=float, default=1e-5)
    parser.add_argument("--gyro-bias-noise", type=float, default=1e-5)
    parser.add_argument(
        "--vel-q-scale",
        type=float,
        default=10.0,
        help=(
            "Scale applied to velocity process noise block Q[3:6,3:6] "
            "(base 0.01 m^2/s^2)"
        ),
    )
    parser.add_argument(
        "--vel-r",
        type=float,
        default=0.25,
        help=(
            "Diagonal variance for GNSS velocity measurements R[3:6,3:6] "
            "[m^2/s^2]"
        ),
    )
    parser.add_argument(
        "--zupt-acc-var",
        type=float,
        default=0.01,
        help="Accelerometer variance threshold for ZUPT detection",
    )
    parser.add_argument(
        "--zupt-gyro-var",
        type=float,
        default=1e-6,
        help="Gyroscope variance threshold for ZUPT detection",
    )
    parser.add_argument(
        "--static-start",
        type=int,
        default=None,
        help="Start sample for bias window when known",
    )
    parser.add_argument(
        "--static-end",
        type=int,
        default=None,
        help="End sample for bias window when known",
    )
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
    imu_file, gnss_file = check_files(args.imu_file, args.gnss_file)
    truth_file = args.truth_file

    os.makedirs("results", exist_ok=True)

    imu_stem = Path(imu_file).stem
    gnss_stem = Path(gnss_file).stem
    tag = TAG(imu=imu_stem, gnss=gnss_stem, method=method)
    summary_tag = f"{imu_stem}_{gnss_stem}"
    dataset_id = imu_stem

    logging.info(f"Running attitude-estimation method: {method}")

    truth_quat0 = None
    if truth_file:
        try:
            truth_first = np.loadtxt(truth_file, comments="#", max_rows=1)
            truth_quat0 = truth_first[8:12]
        except Exception:
            truth_quat0 = None

    # ================================
    # TASK 1: Define Reference Vectors in NED Frame
    # ================================
    logging.info(f"TASK 1 ({method}): Define reference vectors in NED frame")

    (
        lat_deg,
        lon_deg,
        alt,
        g_NED,
        omega_ie_NED,
        mag_NED,
        initial_vel_ned,
        ecef_origin,
    ) = compute_reference_vectors(gnss_file, args.mag_file)
    lat = np.deg2rad(lat_deg)
    _lon = np.deg2rad(lon_deg)
    logging.info(
        f"Computed initial latitude: {lat_deg:.6f}°, longitude: {lon_deg:.6f}° from GNSS"
    )
    logging.info("==== Reference Vectors in NED Frame ====")
    logging.info(f"Gravity vector (NED):        {g_NED} m/s^2")
    logging.info(f"Earth rotation rate (NED):   {omega_ie_NED} rad/s")
    logging.info(f"Latitude (deg):              {lat_deg:.6f}")
    logging.info(f"Longitude (deg):             {lon_deg:.6f}")

    if not args.no_plots:
        try:
            import cartopy.crs as ccrs
        except Exception as e:  # pragma: no cover - optional
            logging.warning(f"cartopy not available, skipping map generation: {e}")
        else:
            fig = plt.figure(figsize=(10, 5))
            ax = fig.add_subplot(1, 1, 1, projection=ccrs.PlateCarree())
            ax.set_global()
            ax.stock_img()
            ax.plot(lon_deg, lat_deg, "ro", markersize=10, transform=ccrs.PlateCarree())
            ax.text(
                lon_deg + 3,
                lat_deg,
                f"{dataset_id}\nLat: {lat_deg:.4f}°\nLon: {lon_deg:.4f}°",
                transform=ccrs.PlateCarree(),
            )
            fig.suptitle("Task 1: Initial Location on Earth")
            fig.tight_layout()
            out = Path("results") / f"{tag}_task1_location_map"
            save_plot_all(fig, str(out), formats=(".png",))
            plt.show()
            plt.close(fig)
            logging.info("Location map saved")
    else:
        logging.info("Skipping plot generation (--no-plots)")

    # ================================
    # TASK 2: Measure the Vectors in the Body Frame
    # ================================
    logging.info("TASK 2: Measure the vectors in the body frame")

    dt_imu, g_body, omega_ie_body, mag_body = measure_body_vectors(
        imu_file,
        static_start=args.static_start,
        static_end=args.static_end,
        mag_file=args.mag_file,
    )
    logging.info(f"Estimated IMU dt: {dt_imu:.6f} s")
    logging.info(f"Gravity vector (body): {g_body}")
    logging.info(f"Earth rotation (body): {omega_ie_body}")

    # ================================
    # TASK 3: Solve Wahba’s Problem
    # ================================

    # ================================
    # TASK 3: Solve Wahba’s Problem
    # ================================
    logging.info(
        "TASK 3: Solve Wahba’s problem (find initial attitude from body to NED)"
    )

    # --------------------------------
    # Subtask 3.1: Prepare Vector Pairs for Attitude Determination
    # --------------------------------
    logging.info("Subtask 3.1: Preparing vector pairs for attitude determination.")
    # Case 1: Current implementation vectors
    v1_B = g_body / np.linalg.norm(g_body)  # Normalize gravity in body frame
    v2_B = (
        omega_ie_body / np.linalg.norm(omega_ie_body)
        if np.linalg.norm(omega_ie_body) > 1e-10
        else np.array([1.0, 0.0, 0.0])
    )
    v1_N = g_NED / np.linalg.norm(g_NED)  # Normalize gravity in NED frame
    v2_N = omega_ie_NED / np.linalg.norm(omega_ie_NED)  # Normalize Earth rotation rate
    logging.debug(f"Case 1 - Normalized body gravity: {v1_B}")
    logging.debug(f"Case 1 - Normalized body Earth rate: {v2_B}")
    logging.debug(f"Case 1 - Normalized NED gravity: {v1_N}")
    logging.debug(f"Case 1 - Normalized NED Earth rate: {v2_N}")

    # Case 2: Recompute ω_ie,NED using document equation
    omega_ie_NED_doc = EARTH_RATE * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    v2_N_doc = omega_ie_NED_doc / np.linalg.norm(
        omega_ie_NED_doc
    )  # Normalize for Case 2
    logging.debug(f"Case 2 - Normalized NED Earth rate (document equation): {v2_N_doc}")

    # --------------------------------
    # Subtask 3.2: TRIAD Method
    # --------------------------------
    logging.info("Subtask 3.2: Computing rotation matrix using TRIAD method.")
    R_tri = triad_svd(v1_B, v2_B, v1_N, v2_N)
    logging.info("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)
    logging.debug("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)

    R_tri_doc = triad_svd(v1_B, v2_B, v1_N, v2_N_doc)
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
    q_dav = np.array(
        [q_dav[0], -q_dav[1], -q_dav[2], -q_dav[3]]
    )  # Conjugate for body-to-NED
    qw, qx, qy, qz = q_dav
    R_dav = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ]
    )
    logging.info("Rotation matrix (Davenport’s Q-Method, Case 1):\n%s", R_dav)
    logging.info("Davenport quaternion (q_w, q_x, q_y, q_z, Case 1): %s", q_dav)
    logging.debug("Rotation matrix (Davenport’s Q-Method, Case 1):\n%s", R_dav)
    logging.debug("Davenport quaternion (Case 1): %s", q_dav)

    # Case 2
    B_doc = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N_doc, v2_B)
    sigma_doc = np.trace(B_doc)
    S_doc = B_doc + B_doc.T
    Z_doc = np.array(
        [
            B_doc[1, 2] - B_doc[2, 1],
            B_doc[2, 0] - B_doc[0, 2],
            B_doc[0, 1] - B_doc[1, 0],
        ]
    )
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
    R_dav_doc = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ]
    )
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
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
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
    euler_tri = R.from_matrix(R_tri).as_euler("xyz", degrees=True)
    logging.info(
        f"TRIAD initial attitude (deg): roll={euler_tri[0]:.3f} pitch={euler_tri[1]:.3f} yaw={euler_tri[2]:.3f}"
    )
    with open("triad_init_log.txt", "a") as logf:
        logf.write(f"{imu_file}: init_euler_deg={euler_tri}\n")
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
    grav_err_max = float(np.max(list(grav_errors.values())))
    omega_err_mean = float(np.mean(list(omega_errors.values())))
    omega_err_max = float(np.max(list(omega_errors.values())))

    # --------------------------------
    # Subtask 3.6: Validate Attitude Determination and Compare Methods
    # --------------------------------
    logging.info(
        "Subtask 3.6: Validating attitude determination and comparing methods."
    )
    # -- Composite quaternion and per-method errors ---------------------------
    quats_case1 = {"TRIAD": q_tri, "Davenport": q_dav, "SVD": q_svd}
    quats_case2 = {"TRIAD": q_tri_doc, "Davenport": q_dav_doc, "SVD": q_svd_doc}

    methods = ["TRIAD", "Davenport", "SVD"]

    def normalise(q):
        return q / np.linalg.norm(q)

    def attitude_errors(q1, q2):
        def quat_to_rot(q):
            w, x, y, z = q
            return np.array(
                [
                    [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
                ]
            )

        R1 = quat_to_rot(q1)
        R2 = quat_to_rot(q2)
        g_vec = np.array([0.0, 0.0, 1.0])
        w_vec = np.array([1.0, 0.0, 0.0])

        def ang(a, b):
            dot = np.clip(
                np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0
            )
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
        "TRIAD": omega_errors["TRIAD"],
        "Davenport": omega_errors["Davenport"],
        "SVD": omega_errors["SVD"],
    }
    diff = max(omega_errs.values()) - min(omega_errs.values())
    tol = 1e-5  # allow up to 0.00001° of spread without complaint

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
    logging.info(
        f"{'Method':10s}  {'Gravity Err (deg)':>18s}  {'Earth-Rate Err (deg)':>22s}"
    )
    for m in ["TRIAD", "Davenport", "SVD"]:
        g = grav_errors[m]
        o = omega_errors[m]
        logging.info(f"{m:10s}  {g:18.4f}  {o:22.4f}")

    # --------------------------------
    # Subtask 3.7: Plot Validation Errors and Quaternion Components
    # --------------------------------

    logging.info("Subtask 3.7: Plotting validation errors and quaternion components.")

    methods_plot = ["TRIAD", "Davenport", "SVD"]
    grav_vals = np.array([grav_errors[m] for m in methods_plot], dtype=float)
    erate_vals = np.array([omega_errors[m] for m in methods_plot], dtype=float)

    if not np.isfinite(grav_vals).all():
        logging.warning("Task3:grav NaN -> 0")
        grav_vals = np.nan_to_num(grav_vals, nan=0.0, posinf=0.0, neginf=0.0)
    if not np.isfinite(erate_vals).all():
        logging.warning("Task3:erate NaN -> 0")
        erate_vals = np.nan_to_num(erate_vals, nan=0.0, posinf=0.0, neginf=0.0)

    logging.info(
        "[Task3] Gravity errors (deg): TRIAD=%.6g, Davenport=%.6g, SVD=%.6g",
        *grav_vals
    )
    logging.info(
        "[Task3] Earth-rate errors (deg): TRIAD=%.6g, Davenport=%.6g, SVD=%.6g",
        *erate_vals
    )

    assert grav_vals.size and erate_vals.size, "Task3:NoData: No error data computed for plotting."

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].bar(methods_plot, grav_vals)
    axes[0].set_title("Gravity Error")
    axes[0].set_ylabel("Error (degrees)")
    ymax = np.max(np.abs(grav_vals))
    if ymax == 0:
        ymax = 0.01
    axes[0].set_ylim(-1.1 * ymax, 1.1 * ymax)
    axes[0].grid(True)
    for i, v in enumerate(grav_vals):
        axes[0].text(i, v, f"{v:.3g}", ha="center", va="bottom")

    axes[1].bar(methods_plot, erate_vals)
    axes[1].set_title("Earth Rate Error")
    axes[1].set_ylabel("Error (degrees)")
    ymax = np.max(np.abs(erate_vals))
    if ymax == 0:
        ymax = 0.01
    axes[1].set_ylim(-1.1 * ymax, 1.1 * ymax)
    axes[1].grid(True)
    for i, v in enumerate(erate_vals):
        axes[1].text(i, v, f"{v:.3g}", ha="center", va="bottom")

    fig.suptitle("Task 3: Attitude Error Comparison")
    fig.tight_layout()
    if not args.no_plots:
        outbase = f"results/{tag}_task3_errors_comparison"
        fig.savefig(outbase + ".pdf")
        fig.savefig(outbase + ".png", dpi=300)
    plt.close(fig)
    logging.info("Error comparison plot saved")

    # Collect quaternion data for both cases
    cases = ["Case 1", "Case 2"]

    # Plot quaternion components
    fig, ax = plt.subplots(figsize=(12, 6))
    labels = [
        f"{m} ({c})" for c in cases for m in methods
    ]  # e.g., 'TRIAD (Case 1)', 'TRIAD (Case 2)', etc.
    x = np.arange(len(labels))
    width = 0.15
    components = ["qw", "qx", "qy", "qz"]

    # Flatten quaternion data for plotting
    all_quats = [quats_case1[m] for m in methods] + [quats_case2[m] for m in methods]

    # Plot bars for each quaternion component
    for i, comp in enumerate(components):
        comp_values = [q[i] for q in all_quats]
        ax.bar(x + i * width, comp_values, width, label=comp)

    ax.set_xticks(x + 1.5 * width)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Quaternion Component Value")
    ax.set_title("Quaternion Components for Each Method and Case")
    fig.suptitle("Task 3: Quaternion Components by Method")
    ax.legend(loc="best")

    plt.tight_layout()
    if not args.no_plots:
        save_plot_all(fig, f"results/{tag}_task3_quaternions_comparison", formats=(".png",))
        plt.show()
    plt.close()
    logging.info("Quaternion comparison plot saved")

    if truth_quat0 is not None and not args.no_plots:
        fig_truth, ax_truth = plt.subplots(figsize=(8, 4))
        ax_truth.bar(range(4), quats_case1["TRIAD"], width=0.4, label="Computed")
        ax_truth.bar(
            np.arange(4) + 0.4,
            truth_quat0,
            width=0.4,
            label="Truth",
        )
        ax_truth.set_xticks(np.arange(4) + 0.2)
        ax_truth.set_xticklabels(["qw", "qx", "qy", "qz"])
        ax_truth.set_ylabel("Value")
        ax_truth.set_title("Task 3: Quaternion vs. Truth")
        ax_truth.legend(loc="best")
        save_plot_all(fig_truth, f"results/{tag}_task3_quaternions_truth", formats=(".png",))
        plt.show()
        plt.close(fig_truth)

    # --------------------------------
    # Subtask 3.8: Store Rotation Matrices for Later Tasks
    # --------------------------------
    logging.info("Subtask 3.8: Storing rotation matrices for use in later tasks.")
    task3_results = {
        "TRIAD": {"R": R_tri},
        "Davenport": {"R": R_dav},
        "SVD": {"R": R_svd},
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
    C_B_N_methods = {m: task3_results[m]["R"] for m in methods}
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
    time_col = "Posix_Time"
    pos_cols = ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]
    vel_cols = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
    gnss_time = zero_base_time(gnss_data[time_col].values)
    gnss_pos_ecef = gnss_data[pos_cols].values
    gnss_vel_ecef = gnss_data[vel_cols].values
    logging.info(f"GNSS data shape: {gnss_pos_ecef.shape}")

    lat_series = []
    lon_series = []
    for x, y, z in gnss_pos_ecef:
        lat_d, lon_d, _ = ecef_to_geodetic(x, y, z)
        lat_series.append(np.deg2rad(lat_d))
        lon_series.append(np.deg2rad(lon_d))
    lat_series = np.array(lat_series)
    lon_series = np.array(lon_series)

    # --------------------------------
    # Subtask 4.5: Define Reference Point
    # --------------------------------
    logging.info("Subtask 4.5: Defining reference point.")
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = ecef_origin
    logging.info(
        f"Reference point: lat={ref_lat:.6f} rad, lon={ref_lon:.6f} rad, r0={ref_r0}"
    )

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
    logging.info(f"GNSS velocity incorporated: {gnss_vel_ned[0]}")
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
    logging.info(
        "Subtask 4.9: Loading IMU data and correcting for bias for each method."
    )
    try:
        imu_data = pd.read_csv(imu_file, sep=r"\s+", header=None)
        imu_time = np.arange(len(imu_data)) * dt_imu
        lat_interp = interpolate_series(imu_time, gnss_time, lat_series)
        lon_interp = interpolate_series(imu_time, gnss_time, lon_series)

        # Convert velocity increments to acceleration (m/s²)
        # Columns 5,6,7 are velocity increments (m/s) over dt_imu
        acc_body = imu_data[[5, 6, 7]].values / dt_imu  # acc_body = delta_v / dt_imu
        gyro_body = (
            imu_data[[2, 3, 4]].values / dt_imu
        )  # gyro_body = delta_theta / dt_imu
        acc_body = butter_lowpass_filter(acc_body)
        gyro_body = butter_lowpass_filter(gyro_body)

        start_idx = args.static_start
        end_idx = args.static_end
        if start_idx is None and end_idx is None:
            dataset_window = {
                "IMU_X001.dat": (296, 479907),
                "IMU_X002.dat": (296, 479907),
                "IMU_X003.dat": (296, 479907),
            }.get(Path(imu_file).name)
            if dataset_window and len(acc_body) >= dataset_window[1]:
                start_idx, end_idx = dataset_window
                end_idx = min(end_idx, len(acc_body))

        if start_idx is not None:
            if end_idx is None:
                end_idx = len(acc_body)
            start_idx = max(0, start_idx)
            end_idx = min(end_idx, len(acc_body))
            N_static = end_idx - start_idx
        else:
            N_static = min(4000, len(imu_data))
            start_idx = 0
            end_idx = N_static

        if N_static < MIN_STATIC_SAMPLES:
            raise ValueError(
                f"Insufficient static samples for bias estimation; require at least {MIN_STATIC_SAMPLES}."
            )

        static_acc, static_gyro = compute_biases(
            acc_body,
            gyro_body,
            start_idx,
            end_idx,
        )

        # Compute accelerometer scale factor using magnitude ratio
        scale = np.linalg.norm(g_NED) / np.linalg.norm(static_acc)

        # Compute corrected acceleration and gyroscope data for each method
        acc_body_corrected = {}
        gyro_body_corrected = {}
        for m in methods:
            C_N_B = C_B_N_methods[m].T  # NED to Body rotation matrix
            g_body_expected = C_N_B @ g_NED  # Expected gravity in body frame

            # Accelerometer bias: static_acc should equal -g_body_expected
            acc_bias = static_acc + g_body_expected  # measured minus expected

            # Gyroscope bias: static_gyro should equal C_N_B @ omega_ie_NED
            omega_ie_NED = np.array(
                [EARTH_RATE * np.cos(ref_lat), 0.0, -EARTH_RATE * np.sin(ref_lat)]
            )
            omega_ie_body_expected = C_N_B @ omega_ie_NED
            gyro_bias = (
                static_gyro - omega_ie_body_expected
            )  # Bias = measured - expected

            # Correct the entire dataset
            acc_body_corrected[m] = scale * (acc_body - acc_bias)
            gyro_body_corrected[m] = gyro_body - gyro_bias
            acc_biases[m] = acc_bias
            gyro_biases[m] = gyro_bias

            bias_mag = np.linalg.norm(acc_bias)
            logging.info(
                f"Method {m}: Accelerometer bias: {acc_bias} (|b|={bias_mag:.6f} m/s^2)"
            )
            logging.info(f"Method {m}: Gyroscope bias: {gyro_bias}")
            logging.info(f"Method {m}: Accelerometer scale factor: {scale:.4f}")
            logging.debug(f"Method {m}: Accelerometer bias: {acc_bias}")
            logging.debug(f"Method {m}: Gyroscope bias: {gyro_bias}")

        logging.info("IMU data corrected for bias for each method.")
        if methods:
            logging.info(
                "Accelerometer scale factor applied: %.4f",
                scale,
            )
            print(f"Task 4: applied accelerometer scale factor = {scale:.4f}")
    except Exception as e:
        logging.error(f"Failed to load IMU data or compute corrections: {e}")
        raise

    # --------------------------------
    # Subtask 4.10: Set IMU Parameters and Gravity Vector
    # --------------------------------
    logging.info("Subtask 4.10: Setting IMU parameters and gravity vector.")
    # Use the gravity vector computed in Task 1 instead of overwriting with the
    # constant defined in ``constants.GRAVITY``.  This preserves any
    # location-specific variation calculated earlier in the pipeline.
    logging.info(f"Using gravity vector from Task 1: {g_NED}")

    # --------------------------------
    # Subtask 4.11: Initialize Output Arrays
    # --------------------------------
    logging.info("Subtask 4.11: Initializing output arrays.")
    # per-method integration results
    pos_integ = {}
    vel_integ = {}
    acc_integ = {}
    pos_integ_ecef = {}
    vel_integ_ecef = {}

    # --------------------------------
    # Subtask 4.12: Integrate IMU Accelerations for Each Method
    # --------------------------------
    logging.info("Subtask 4.12: Integrating IMU accelerations for each method.")
    for m in methods:
        logging.info(f"Integrating IMU data using {m} method.")
        C_B_N = C_B_N_methods[m]
        pos, vel, acc, pos_e, vel_e = integrate_trajectory(
            acc_body_corrected[m],
            imu_time,
            C_B_N,
            g_NED,
            lat=lat_interp,
            lon=lon_interp,
            ref_lat=ref_lat,
            ref_lon=ref_lon,
            ref_ecef=ref_r0,
            debug=args.verbose,
        )
        pos_integ[m] = pos
        vel_integ[m] = vel
        acc_integ[m] = acc
        pos_integ_ecef[m] = pos_e
        vel_integ_ecef[m] = vel_e
    logging.info(
        "IMU-derived position, velocity, and acceleration computed for all methods."
    )

    # --------------------------------
    # Subtask 4.13: Validate and Plot Data
    # --------------------------------
    logging.info("Subtask 4.13: Validating and plotting data.")
    t0 = gnss_time[0]
    t_rel_ilu = imu_time - t0
    t_rel_gnss = gnss_time - t0
    truth_pos_ecef_i = truth_vel_ecef_i = None
    truth_pos_ned_i = truth_vel_ned_i = None
    t_truth = pos_truth_ecef = vel_truth_ecef = None
    if truth_file:
        try:
            truth = np.loadtxt(truth_file, comments="#")
            t_truth = zero_base_time(truth[:, 1])
            pos_truth_ecef = truth[:, 2:5]
            vel_truth_ecef = truth[:, 5:8]
            truth_pos_ecef_i = interpolate_series(t_rel_ilu, t_truth, pos_truth_ecef)
            truth_vel_ecef_i = interpolate_series(t_rel_ilu, t_truth, vel_truth_ecef)
            truth_pos_ned_i = ecef_to_ned(truth_pos_ecef_i, ref_lat, ref_lon, ref_r0)
            truth_vel_ned_i = (C_ECEF_to_NED @ truth_vel_ecef_i.T).T
        except Exception as e:
            logging.error(f"Failed to load truth file {truth_file}: {e}")
            truth_file = None
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
    directions = ["North", "East", "Down"]
    colors = COLORS
    for j in range(3):
        # Position comparison
        ax = axes_comp[0, j]
        ax.plot(t_rel_gnss, gnss_pos_ned[:, j], "k--", label="Measured GNSS")
        for m in methods:
            c = colors.get(m, None)
            ax.plot(
                t_rel_ilu,
                pos_integ[m][:, j],
                color=c,
                alpha=0.7,
                label=f"Derived IMU ({m})",
            )
        ax.set_title(f"Position {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.legend(loc="best")

        # Velocity comparison
        ax = axes_comp[1, j]
        ax.plot(t_rel_gnss, gnss_vel_ned[:, j], "k--", label="Measured GNSS")
        for m in methods:
            c = colors.get(m, None)
            ax.plot(
                t_rel_ilu,
                vel_integ[m][:, j],
                color=c,
                alpha=0.7,
                label=f"Derived IMU ({m})",
            )
        ax.set_title(f"Velocity {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend(loc="best")

        # Acceleration comparison
        ax = axes_comp[2, j]
        ax.plot(t_rel_gnss, gnss_acc_ned[:, j], "k--", label="Derived GNSS")
        for m in methods:
            c = colors.get(m, None)
            ax.plot(
                t_rel_ilu,
                acc_integ[m][:, j],
                color=c,
                alpha=0.7,
                label=f"Derived IMU ({m})",
            )
        ax.set_title(f"Acceleration {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (m/s²)")
        ax.legend(loc="best")
    fig_comp.suptitle(f"Task 4 – {method} – NED Frame (IMU vs. GNSS)")
    fig_comp.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot_all(fig_comp, f"results/{tag}_task4_all_ned", formats=(".png",))
        plt.show()
    plt.close(fig_comp)
    logging.info("NED frame plot saved")

    # Plot 1: Data in mixed frames (GNSS position/velocity in ECEF, IMU acceleration in body)
    # Plot in ECEF frame
    logging.info("Plotting all data in ECEF frame.")

    # Plot 3: All data in ECEF frame
    logging.info("Plotting all data in ECEF frame.")
    fig_ecef, axes_ecef = plt.subplots(3, 3, figsize=(15, 10))
    directions_ecef = ["X", "Y", "Z"]
    for i in range(3):
        for j in range(3):
            ax = axes_ecef[i, j]
            if i == 0:  # Position
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], "k-", label="Measured GNSS")
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        pos_integ_ecef[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Position {directions_ecef[j]}_ECEF")
            elif i == 1:  # Velocity
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], "k-", label="Measured GNSS")
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        vel_integ_ecef[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Velocity V{directions_ecef[j]}_ECEF")
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                    f_ecef = C_NED_to_ECEF @ f_ned
                    ax.plot(
                        t_rel_ilu,
                        f_ecef[j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Acceleration A{directions_ecef[j]}_ECEF")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ecef.suptitle(f"Task 4 – {method} – ECEF Frame (IMU vs. GNSS)")
    fig_ecef.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot_all(fig_ecef, f"results/{tag}_task4_all_ecef", formats=(".png",))
        plt.show()
    plt.close(fig_ecef)
    logging.info("All data in ECEF frame plot saved")

    # Plot 4: All data in body frame
    logging.info("Plotting all data in body frame.")
    fig_body, axes_body = plt.subplots(3, 3, figsize=(15, 10))
    directions_body = ["X", "Y", "Z"]
    for i in range(3):
        for j in range(3):
            ax = axes_body[i, j]
            if i == 0:  # Position
                r_body = (C_N_B @ gnss_pos_ned.T).T
                ax.plot(t_rel_gnss, r_body[:, j], "k-", label="Measured GNSS")
                ax.set_title(f"Position r{directions_body[j]}_body")
            elif i == 1:  # Velocity
                vel_body = (C_N_B @ gnss_vel_ned.T).T
                ax.plot(t_rel_gnss, vel_body[:, j], "k-", label="Measured GNSS")
                ax.set_title(f"Velocity v{directions_body[j]}_body")
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        acc_body_corrected[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Acceleration A{directions_body[j]}_body")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_body.suptitle(f"Task 4 – {method} – Body Frame (IMU vs. GNSS)")
    fig_body.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot_all(fig_body, f"results/{tag}_task4_all_body", formats=(".png",))
        plt.show()
    plt.close(fig_body)
    logging.info("All data in body frame plot saved")

    # ================================
    # Task 5: Sensor Fusion with Kalman Filter
    # ================================
    logging.info("Task 5: Sensor Fusion with Kalman Filter")

    # --------------------------------
    # Subtask 5.1: Configure Logging
    # --------------------------------
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
        imu_data = pd.read_csv(imu_file, sep=r"\s+", header=None)
    except FileNotFoundError as e:
        missing = "GNSS" if "csv" in str(e) else "IMU"
        logging.error(f"{missing} file not found: {e.filename}")
        raise
    except Exception as e:
        logging.error(f"Failed to load data: {e}")
        raise

    # Extract GNSS fields
    time_col = "Posix_Time"
    pos_cols = ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]
    vel_cols = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
    gnss_time = zero_base_time(gnss_data[time_col].values)
    gnss_pos_ecef = gnss_data[pos_cols].values
    gnss_vel_ecef = gnss_data[vel_cols].values

    # Reference position for NED
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = ecef_origin
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
    imu_time = np.arange(len(imu_data)) * dt_imu
    acc_body = imu_data[[5, 6, 7]].values / dt_imu
    acc_body = butter_lowpass_filter(acc_body)
    # Use at most 4000 samples but allow shorter sequences when running the
    # trimmed datasets used in unit tests.
    N_static = min(4000, len(imu_data))
    if N_static < MIN_STATIC_SAMPLES:
        raise ValueError(
            f"Insufficient static samples; require at least {MIN_STATIC_SAMPLES}."
        )
    static_acc = np.mean(acc_body[:N_static], axis=0)

    # Estimate scale factor using bias-removed static acceleration
    C_N_B_ref = C_B_N_methods[methods[0]].T
    g_body_expected_ref = C_N_B_ref @ g_NED
    bias_ref = static_acc + g_body_expected_ref
    g_body_mag = np.linalg.norm(static_acc - bias_ref)
    scale = np.linalg.norm(g_NED) / g_body_mag

    # Compute corrected acceleration for each method
    acc_body_corrected = {}
    for m in methods:
        C_N_B = C_B_N_methods[m].T
        g_body_expected = C_N_B @ g_NED
        bias = static_acc + g_body_expected
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
            dt = imu_time[i] - imu_time[i - 1]
            f_ned = C_B_N @ acc_body_corrected[m][i]
            a_ned = f_ned + g_NED
            imu_acc[m][i] = a_ned
            imu_vel[m][i] = (
                imu_vel[m][i - 1] + 0.5 * (imu_acc[m][i] + imu_acc[m][i - 1]) * dt
            )
            imu_pos[m][i] = (
                imu_pos[m][i - 1] + 0.5 * (imu_vel[m][i] + imu_vel[m][i - 1]) * dt
            )
        logging.info(f"Method {m}: IMU data integrated.")
        final_vel = imu_vel[m][-1]
        logging.info(
            f"[{summary_tag} | {m}] Final integrated NED velocity: "
            f"[{final_vel[0]:.3f}, {final_vel[1]:.3f}, {final_vel[2]:.3f}] m/s"
        )

    # ZUPT handled dynamically during Kalman filtering

    # --------------------------------
    # Subtask 5.6: Kalman Filter for Sensor Fusion for Each Method
    # --------------------------------
    logging.info(
        "Subtask 5.6: Running Kalman Filter for sensor fusion for each method."
    )
    fused_pos = {m: np.zeros_like(imu_pos[m]) for m in methods}
    fused_vel = {m: np.zeros_like(imu_vel[m]) for m in methods}
    fused_acc = {m: np.zeros_like(imu_acc[m]) for m in methods}
    # Interpolate GNSS data to IMU time once
    gnss_pos_ned_interp = interpolate_series(imu_time, gnss_time, gnss_pos_ned)
    gnss_vel_ned_interp = interpolate_series(imu_time, gnss_time, gnss_vel_ned)
    gnss_acc_ned_interp = interpolate_series(imu_time, gnss_time, gnss_acc_ned)
    logging.debug(
        "Interpolated GNSS data: first NED pos %.4f last %.4f",
        gnss_pos_ned_interp[0, 0],
        gnss_pos_ned_interp[-1, 0],
    )

    innov_pos_all = {}
    innov_vel_all = {}
    attitude_q_all = {}
    euler_all = {}
    res_pos_all = {}
    res_vel_all = {}
    time_res_all = {}
    P_hist_all = {}
    x_log_all = {}
    zupt_counts = {}
    zupt_events_all = {}

    for m in methods:
        kf = KalmanFilter(dim_x=13, dim_z=6)
        initial_quats = {"TRIAD": q_tri, "Davenport": q_dav, "SVD": q_svd}
        kf.x = np.hstack(
            (imu_pos[m][0], imu_vel[m][0], imu_acc[m][0], initial_quats[m])
        )
        kf.F = np.eye(13)
        kf.H = np.hstack((np.eye(6), np.zeros((6, 7))))
        kf.P *= 1.0
        kf.R = np.eye(6) * 0.1  # position/velocity measurement variance [m^2/s^2]
        kf.Q = np.eye(13) * 0.01  # process noise base [m^2/s^2]
        kf.Q[3:6, 3:6] *= args.vel_q_scale
        kf.R[3:6, 3:6] = np.eye(3) * args.vel_r
        logging.info(f"Adjusted Q[3:6,3:6]: {kf.Q[3:6,3:6]}")
        logging.info(f"Adjusted R[3:6,3:6]: {kf.R[3:6,3:6]}")
        fused_pos[m][0] = imu_pos[m][0]
        fused_vel[m][0] = imu_vel[m][0]
        fused_acc[m][0] = imu_acc[m][0]

        # ---------- logging containers ----------
        innov_pos = []  # GNSS - predicted position
        innov_vel = []  # GNSS - predicted velocity
        attitude_q = []  # quaternion history
        res_pos_list, res_vel_list, time_res = [], [], []
        euler_list = []

        win = 80
        acc_win = []
        gyro_win = []
        zupt_count = 0
        zupt_events = []
        vel_blow_count = 0  # track number of velocity blow-ups
        vel_blow_warn_interval = 0  # set >0 to re-warn every N events
        P_hist = [kf.P.copy()]

        # attitude initialisation for logging
        orientations = np.zeros((len(imu_time), 4))
        orientations[0] = initial_quats[m]
        attitude_q.append(orientations[0])
        q_cur = orientations[0]
        roll, pitch, yaw = quat2euler(q_cur)
        euler_list.append([roll, pitch, yaw])

        # State history log (states x time)
        x_log = np.zeros((kf.dim_x, len(imu_time)))
        x_log[:, 0] = kf.x

        # Run Kalman Filter
        for i in range(1, len(imu_time)):
            dt = imu_time[i] - imu_time[i - 1]

            # propagate quaternion using gyro measurement
            dq = quat_from_rate(gyro_body_corrected[m][i], dt)
            q_cur = quat_multiply(q_cur, dq)
            q_cur /= np.linalg.norm(q_cur)
            orientations[i] = q_cur
            kf.x[9:13] = q_cur

            # Prediction step
            kf.F[0:3, 3:6] = np.eye(3) * dt
            kf.F[3:6, 6:9] = np.eye(3) * dt
            kf.predict()

            if np.linalg.norm(kf.x[3:6]) > 500:
                vel_blow_count += 1
                if vel_blow_count == 1 or (
                    vel_blow_warn_interval > 0
                    and vel_blow_count % vel_blow_warn_interval == 0
                ):
                    logging.warning(
                        "Velocity blew up (%.1f m/s); zeroing Δv and continuing.",
                        np.linalg.norm(kf.x[3:6]),
                    )
                kf.x[3:6] = 0.0

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
            if len(acc_win) == win and is_static(
                np.array(acc_win),
                np.array(gyro_win),
                accel_var_thresh=args.zupt_acc_var,
                gyro_var_thresh=args.zupt_gyro_var,
            ):
                H_z = np.zeros((3, 13))
                H_z[:, 3:6] = np.eye(3)
                R_z = np.eye(3) * 1e-4
                pred_v = H_z @ kf.x
                S = H_z @ kf.P @ H_z.T + R_z
                K = kf.P @ H_z.T @ np.linalg.inv(S)
                kf.x += K @ (-pred_v)
                kf.P = (np.eye(13) - K @ H_z) @ kf.P
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
            x_log[:, i] = kf.x

        logging.info(
            f"Method {m}: Kalman Filter completed. ZUPTcnt={zupt_count}"
        )
        logging.info(
            f"Method {m}: velocity blow-up events={vel_blow_count}"
        )
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
        x_log_all[m] = x_log

        if m == "Davenport":
            pos_finite = np.all(np.isfinite(fused_pos[m]), axis=0)
            vel_finite = np.all(np.isfinite(fused_vel[m]), axis=0)
            if not pos_finite.all() or not vel_finite.all():
                logging.warning(
                    "Non-finite values in Davenport fused results: "
                    f"pos_finite={pos_finite}, vel_finite={vel_finite}"
                )
            else:
                logging.debug(
                    "Davenport fused position stats: "
                    f"min={np.min(fused_pos[m], axis=0)}, "
                    f"max={np.max(fused_pos[m], axis=0)}"
                )
                logging.debug(
                    "Davenport fused velocity stats: "
                    f"min={np.min(fused_vel[m], axis=0)}, "
                    f"max={np.max(fused_vel[m], axis=0)}"
                )

    # Compute residuals for the selected method
    _ = res_pos_all[method]
    _ = res_vel_all[method]
    _ = time_res_all[method]

    _ = np.rad2deg(euler_all[method])

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
    directions = ["North", "East", "Down"]

    # Subtask 5.8.2: Plotting Results for selected method
    logging.info(f"Subtask 5.8.2: Plotting results for {method}.")
    logging.debug(f"# Subtask 5.8.2: Starting to plot results for {method}.")
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))

    # Davenport - Position
    for j in range(3):
        ax = axes[0, j]
        ax.plot(imu_time, gnss_pos_ned_interp[:, j], "k-", label="GNSS (Measured)")
        ax.plot(imu_time, imu_pos[method][:, j], "g--", label="IMU (Derived)")
        c = colors.get(method, None)
        ax.plot(
            imu_time, fused_pos[method][:, j], c, alpha=0.7, label="Fused GNSS + IMU"
        )
        ax.set_title(f"Position {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.legend(loc="best")
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
        ax.plot(imu_time, gnss_vel_ned_interp[:, j], "k-", label="GNSS (Measured)")
        ax.plot(imu_time, imu_vel[method][:, j], "g--", label="IMU (Derived)")
        c = colors.get(method, None)
        ax.plot(
            imu_time, fused_vel[method][:, j], c, alpha=0.7, label="Fused GNSS + IMU"
        )
        ax.set_title(f"Velocity {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend(loc="best")
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
        ax.plot(imu_time, imu_acc[method][:, j], "g--", label="IMU (Derived)")
        ax.plot(imu_time, gnss_acc_ned_interp[:, j], "k-", label="GNSS (Derived)")
        c = colors.get(method, None)
        ax.plot(
            imu_time, fused_acc[method][:, j], c, alpha=0.7, label="Fused GNSS + IMU"
        )
        ax.set_title(f"Acceleration {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (m/s²)")
        ax.legend(loc="best")
        logging.info(
            f"Subtask 5.8.2: Plotted {method} acceleration {directions[j]}: "
            f"First = {fused_acc[method][0, j]:.4f}, Last = {fused_acc[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} acceleration {directions[j]}: "
            f"First = {fused_acc[method][0, j]:.4f}, Last = {fused_acc[method][-1, j]:.4f}"
        )

    plt.tight_layout()
    out_png = f"results/{tag}_task5_results_ned_{method}.png"
    if not args.no_plots:
        save_plot(fig, out_png, f"Task 5 – {method} – NED Frame")
        plt.show()
    logging.info(f"Subtask 5.8.2: {method} plot saved as '{out_png}'")
    logging.debug(
        f"# Subtask 5.8.2: {method} plotting completed. Saved as '{out_png}'."
    )


    # ----- Additional reference frame plots -----
    logging.info("Plotting all data in NED frame.")
    fig_ned_all, ax_ned_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_ned = ["N", "E", "D"]
    c = colors.get(method, None)
    for i in range(3):
        for j in range(3):
            ax = ax_ned_all[i, j]
            if i == 0:
                ax.plot(t_rel_gnss, gnss_pos_ned[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    fused_pos[method][:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                if truth_pos_ned_i is not None:
                    ax.plot(t_rel_ilu, truth_pos_ned_i[:, j], "m-", label="Truth")
                ax.set_title(f"Position {dirs_ned[j]}")
            elif i == 1:
                ax.plot(t_rel_gnss, gnss_vel_ned[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    fused_vel[method][:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                if truth_vel_ned_i is not None:
                    ax.plot(t_rel_ilu, truth_vel_ned_i[:, j], "m-", label="Truth")
                ax.set_title(f"Velocity V{dirs_ned[j]}")
            else:
                ax.plot(t_rel_gnss, gnss_acc_ned[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    fused_acc[method][:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                ax.set_title(f"Acceleration {dirs_ned[j]}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ned_all.suptitle(f"Task 5 – {method} – NED Frame (Fused vs. Measured GNSS)")
    fig_ned_all.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot_all(fig_ned_all, f"results/{tag}_task5_all_ned", formats=(".png",))
        plt.show()
    plt.close(fig_ned_all)
    logging.info("All data in NED frame plot saved")

    logging.info("Plotting all data in ECEF frame.")
    fig_ecef_all, ax_ecef_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_ecef = ["X", "Y", "Z"]
    pos_ecef = np.array([C_NED_to_ECEF @ p + ref_r0 for p in fused_pos[method]])
    vel_ecef = (C_NED_to_ECEF @ fused_vel[method].T).T
    acc_ecef = (C_NED_to_ECEF @ fused_acc[method].T).T

    for name, arr in (
        ("pos_ecef", pos_ecef),
        ("vel_ecef", vel_ecef),
        ("acc_ecef", acc_ecef),
    ):
        if not np.all(np.isfinite(arr)):
            logging.warning(f"NaNs detected in {name} after NED->ECEF conversion")
        logging.debug(f"{name} min={np.min(arr, axis=0)}, max={np.max(arr, axis=0)}")
    for i in range(3):
        for j in range(3):
            ax = ax_ecef_all[i, j]
            if i == 0:
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    pos_ecef[:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                if truth_pos_ecef_i is not None:
                    ax.plot(t_rel_ilu, truth_pos_ecef_i[:, j], "m-", label="Truth")
                ax.set_title(f"Position {dirs_ecef[j]}_ECEF")
            elif i == 1:
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    vel_ecef[:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                if truth_vel_ecef_i is not None:
                    ax.plot(t_rel_ilu, truth_vel_ecef_i[:, j], "m-", label="Truth")
                ax.set_title(f"Velocity V{dirs_ecef[j]}_ECEF")
            else:
                ax.plot(t_rel_gnss, gnss_acc_ecef[:, j], "k-", label="Derived GNSS")
                ax.plot(
                    t_rel_ilu,
                    acc_ecef[:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                ax.set_title(f"Acceleration {dirs_ecef[j]}_ECEF")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ecef_all.suptitle(f"Task 5 – {method} – ECEF Frame (Fused vs. Measured GNSS)")
    fig_ecef_all.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot_all(fig_ecef_all, f"results/{tag}_task5_all_ecef", formats=(".png",))
        plt.show()
    plt.close(fig_ecef_all)
    logging.info("All data in ECEF frame plot saved")

    logging.info("Plotting all data in body frame.")
    fig_body_all, ax_body_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_body = ["X", "Y", "Z"]
    C_N_B = C_B_N_methods[method].T
    pos_body = (C_N_B @ fused_pos[method].T).T
    vel_body = (C_N_B @ fused_vel[method].T).T
    acc_body = (C_N_B @ fused_acc[method].T).T
    if truth_pos_ned_i is not None:
        truth_pos_body = (C_N_B @ truth_pos_ned_i.T).T
        truth_vel_body = (C_N_B @ truth_vel_ned_i.T).T
    gnss_pos_body = (C_N_B @ gnss_pos_ned.T).T
    gnss_vel_body = (C_N_B @ gnss_vel_ned.T).T
    gnss_acc_body = (C_N_B @ gnss_acc_ned.T).T
    for i in range(3):
        for j in range(3):
            ax = ax_body_all[i, j]
            if i == 0:
                ax.plot(t_rel_gnss, gnss_pos_body[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    pos_body[:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                if truth_pos_ned_i is not None:
                    ax.plot(t_rel_ilu, truth_pos_body[:, j], "m-", label="Truth")
                ax.set_title(f"Position r{dirs_body[j]}_body")
            elif i == 1:
                ax.plot(t_rel_gnss, gnss_vel_body[:, j], "k-", label="Measured GNSS")
                ax.plot(
                    t_rel_ilu,
                    vel_body[:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                if truth_vel_ned_i is not None:
                    ax.plot(t_rel_ilu, truth_vel_body[:, j], "m-", label="Truth")
                ax.set_title(f"Velocity v{dirs_body[j]}_body")
            else:
                ax.plot(t_rel_gnss, gnss_acc_body[:, j], "k-", label="Derived GNSS")
                ax.plot(
                    t_rel_ilu,
                    acc_body[:, j],
                    c,
                    alpha=0.7,
                    label=f"Fused (GNSS+IMU, {method})",
                )
                ax.set_title(f"Acceleration A{dirs_body[j]}_body")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_body_all.suptitle(f"Task 5 – {method} – Body Frame (Fused vs. Measured GNSS)")
    fig_body_all.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot_all(fig_body_all, f"results/{tag}_task5_all_body", formats=(".png",))
        plt.show()
    plt.close(fig_body_all)
    logging.info("All data in body frame plot saved")

    # Plot pre-fit innovations
    # Plot residuals using helper functions
    if not args.no_plots:
        res = compute_residuals(gnss_time, gnss_pos_ned, imu_time, fused_pos[method])
        plot_residuals(gnss_time, res, f"results/residuals_{tag}_{method}.pdf")

    # Create plot summary
    summary = {
        f"{tag}_location_map.pdf": "Initial location on Earth map",
        f"{tag}_task3_errors_comparison.pdf": "Attitude initialization error comparison",
        f"{tag}_task3_quaternions_comparison.pdf": "Quaternion components for initialization",
        f"{tag}_task4_comparison_ned.pdf": "GNSS vs IMU data in NED frame",
        f"{tag}_task4_mixed_frames.pdf": "GNSS/IMU data in mixed frames",
        f"{tag}_task4_all_ned.png": "Integrated data in NED frame",
        f"{tag}_task4_all_ecef.png": "Integrated data in ECEF frame",
        f"{tag}_task4_all_body.png": "Integrated data in body frame",
        f"{tag}_task5_results_{method}.pdf": f"Kalman filter results using {method}",
        f"{tag}_task5_mixed_frames.pdf": "Kalman filter results in mixed frames",
        f"{tag}_task5_all_ned.png": "Kalman filter results in NED frame",
        f"{tag}_task5_all_ecef.png": "Kalman filter results in ECEF frame",
        f"{tag}_task5_all_body.png": "Kalman filter results in body frame",
        f"{tag}_{method.lower()}_residuals.pdf": "Position and velocity residuals",
    }
    summary_path = os.path.join("results", f"{tag}_plot_summary.md")
    with open(summary_path, "w") as f:
        for name, desc in summary.items():
            f.write(f"- **{name}**: {desc}\n")

    rmse_pos = np.sqrt(
        np.mean(np.sum((gnss_pos_ned_interp - fused_pos[method]) ** 2, axis=1))
    )
    final_pos = np.linalg.norm(gnss_pos_ned_interp[-1] - fused_pos[method][-1])

    final_vel_mag = float(np.linalg.norm(fused_vel[method][-1]))
    if final_vel_mag > 500:
        raise RuntimeError(
            f"KF diverged: final velocity {final_vel_mag:.2f} m/s exceeds 500 m/s - check F & H matrices"
        )

    # --- Additional residual metrics ---------------------------------------
    pos_interp = interpolate_series(gnss_time, imu_time, fused_pos[method])
    vel_interp = interpolate_series(gnss_time, imu_time, fused_vel[method])
    resid_pos = pos_interp - gnss_pos_ned
    resid_vel = vel_interp - gnss_vel_ned

    rms_resid_pos = np.sqrt(np.mean(resid_pos**2))
    rms_resid_vel = np.sqrt(np.mean(resid_vel**2))
    max_resid_pos = np.max(np.linalg.norm(resid_pos, axis=1))
    max_resid_vel = np.max(np.linalg.norm(resid_vel, axis=1))

    accel_bias = acc_biases.get(method, np.zeros(3))
    gyro_bias = gyro_biases.get(method, np.zeros(3))

    # --- Attitude angles ----------------------------------------------------
    euler = R.from_quat(attitude_q_all[method]).as_euler("xyz", degrees=True)

    C_NED_to_ECEF = C_ECEF_to_NED.T
    pos_ecef = np.array([C_NED_to_ECEF @ p + ref_r0 for p in fused_pos[method]])
    vel_ecef = (C_NED_to_ECEF @ fused_vel[method].T).T
    C_N_B = C_B_N_methods[method].T
    pos_body = (C_N_B @ fused_pos[method].T).T
    vel_body = (C_N_B @ fused_vel[method].T).T

    np.savez_compressed(
        f"results/{tag}_kf_output.npz",
        summary=dict(
            rmse_pos=rmse_pos,
            final_pos=final_pos,
            grav_err_mean=grav_err_mean,
            grav_err_max=grav_err_max,
            earth_rate_err_mean=omega_err_mean,
            earth_rate_err_max=omega_err_max,
            vel_blow_events=vel_blow_count,
        ),
        time=imu_time,
        pos_ned=fused_pos[method],
        vel_ned=fused_vel[method],
        fused_pos=fused_pos[method],
        fused_vel=fused_vel[method],
        pos_ecef=pos_ecef,
        vel_ecef=vel_ecef,
        truth_pos_ecef=pos_truth_ecef if pos_truth_ecef is not None else np.array([]),
        truth_vel_ecef=vel_truth_ecef if vel_truth_ecef is not None else np.array([]),
        truth_time=t_truth if t_truth is not None else np.array([]),
        pos_body=pos_body,
        vel_body=vel_body,
        innov_pos=innov_pos_all[method],
        innov_vel=innov_vel_all[method],
        euler=euler_all[method],
        residual_pos=res_pos_all[method],
        residual_vel=res_vel_all[method],
        time_residuals=time_res_all[method],
        attitude_q=attitude_q_all[method],
        P_hist=P_hist_all[method],
        x_log=x_log_all[method],
        ref_lat=np.array([ref_lat]),
        ref_lon=np.array([ref_lon]),
        ref_r0=ref_r0,
    )

    # Also export results as MATLAB-compatible .mat for post-processing
    from utils import save_mat

    save_mat(
        f"results/{tag}_kf_output.mat",
        {
            "rmse_pos": np.array([rmse_pos]),
            "final_pos": np.array([final_pos]),
            "grav_err_mean": np.array([grav_err_mean]),
            "grav_err_max": np.array([grav_err_max]),
            "earth_rate_err_mean": np.array([omega_err_mean]),
            "earth_rate_err_max": np.array([omega_err_max]),
            "vel_blow_events": np.array([vel_blow_count]),
            "time": imu_time,
            "pos_ned": fused_pos[method],
            "vel_ned": fused_vel[method],
            "fused_pos": fused_pos[method],
            "fused_vel": fused_vel[method],
            "pos_ecef": pos_ecef,
            "vel_ecef": vel_ecef,
            "truth_pos_ecef": pos_truth_ecef if pos_truth_ecef is not None else np.empty((0, 3)),
            "truth_vel_ecef": vel_truth_ecef if vel_truth_ecef is not None else np.empty((0, 3)),
            "truth_time": t_truth if t_truth is not None else np.empty(0),
            "pos_body": pos_body,
            "vel_body": vel_body,
            "innov_pos": innov_pos_all[method],
            "innov_vel": innov_vel_all[method],
            "euler": euler_all[method],
            "residual_pos": res_pos_all[method],
            "residual_vel": res_vel_all[method],
            "time_residuals": time_res_all[method],
            "attitude_q": attitude_q_all[method],
            "P_hist": P_hist_all[method],
            "x_log": x_log_all[method],
            "ref_lat": np.array([ref_lat]),
            "ref_lon": np.array([ref_lon]),
            "ref_r0": ref_r0,
        },
    )

    # --- Persist for cross-dataset comparison ------------------------------
    import pickle
    import gzip

    pack = {
        "method": method,
        "dataset": summary_tag.split("_")[1],
        "t": t_rel_ilu,
        "pos_ned": fused_pos[method],
        "vel_ned": fused_vel[method],
        "pos_gnss": gnss_pos_ned,
        "vel_gnss": gnss_vel_ned,
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
            zupt_mask[s : e + 1] = True

        save_zupt_variance(
            acc_body_corrected[method],
            zupt_mask,
            dt_imu,
            dataset_id,
            threshold=0.01,
        )

        euler_deg = np.rad2deg(euler_all[method])
        save_euler_angles(imu_time, euler_deg, dataset_id, method)

        pos_f = interpolate_series(gnss_time, imu_time, fused_pos[method])
        vel_f = interpolate_series(gnss_time, imu_time, fused_vel[method])
        save_velocity_profile(gnss_time, vel_f, gnss_vel_ned)
        save_residual_plots(
            gnss_time,
            pos_f,
            gnss_pos_ned,
            vel_f,
            gnss_vel_ned,
            tag,
        )

        plot_all_methods(
            imu_time,
            gnss_pos_ned_interp,
            gnss_vel_ned_interp,
            gnss_acc_ned_interp,
            fused_pos,
            fused_vel,
            fused_acc,
        )

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