import pandas as pd
import numpy as np
import logging
import cartopy.crs as ccrs
import matplotlib.pyplot as plt
import os
import argparse
from pathlib import Path
from paths import GNSS, OUT

# --- plotting output configuration ----------------------------------------
OUT.mkdir(parents=True, exist_ok=True)
os.chdir(OUT)
DATASET_ID = "X001"
METHOD = "TRIAD"
TAG = f"{DATASET_ID}_{METHOD}"

# ---------------------------------------------------------------------------
# Command line configuration
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="GNSS/IMU fusion and attitude initialization"
)
parser.add_argument(
    "--interactive",
    action="store_true",
    help="Display plots interactively instead of closing them",
)
args = parser.parse_args()
INTERACTIVE = args.interactive

from constants import EARTH_RATE
from utils import compute_C_ECEF_to_NED, zero_base_time

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


# Function to convert ECEF to geodetic coordinates
def ecef_to_geodetic(x, y, z):
    """Convert ECEF coordinates to geodetic latitude, longitude, and altitude (WGS-84)."""
    a = 6378137.0  # WGS-84 semi-major axis (meters)
    e_sq = 6.69437999014e-3  # WGS-84 first eccentricity squared
    p = np.sqrt(x**2 + y**2)
    theta = np.arctan2(z * a, p * (1 - e_sq))
    lon = np.arctan2(y, x)
    lat = np.arctan2(
        z + e_sq * a * np.sin(theta) ** 3 / (1 - e_sq),
        p - e_sq * a * np.cos(theta) ** 3,
    )
    N = a / np.sqrt(1 - e_sq * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), alt


# ================================
# TASK 1: Define Reference Vectors in NED Frame
# ================================
logging.info("TASK 1: Define reference vectors in NED frame")

# --------------------------------
# Subtask 1.1: Set Initial Latitude and Longitude from GNSS ECEF Data
# --------------------------------
logging.info("Subtask 1.1: Setting initial latitude and longitude from GNSS ECEF data.")
try:
    gnss_data = pd.read_csv(GNSS / "GNSS_X001.csv")
except Exception as e:
    logging.error(f"Failed to load GNSS data file: {e}")
    raise

# Debug: Print column names and first few rows of ECEF coordinates
print("GNSS data columns:", gnss_data.columns.tolist())
print(
    "First few rows of ECEF coordinates:\n",
    gnss_data[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].head(),
)

# Find first row with non-zero ECEF coordinates
valid_rows = gnss_data[
    (gnss_data["X_ECEF_m"] != 0)
    | (gnss_data["Y_ECEF_m"] != 0)
    | (gnss_data["Z_ECEF_m"] != 0)
]
if not valid_rows.empty:
    initial_row = valid_rows.iloc[0]
    x_ecef = float(initial_row["X_ECEF_m"])
    y_ecef = float(initial_row["Y_ECEF_m"])
    z_ecef = float(initial_row["Z_ECEF_m"])
    lat_deg, lon_deg, alt = ecef_to_geodetic(x_ecef, y_ecef, z_ecef)
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    logging.info(
        f"Computed initial latitude: {lat_deg:.6f}°, longitude: {lon_deg:.6f}° from ECEF coordinates."
    )
    print(f"Initial latitude: {lat_deg:.6f}°, Initial longitude: {lon_deg:.6f}°")
else:
    raise ValueError("No valid ECEF coordinates found in GNSS data.")

# --------------------------------
# Subtask 1.2: Define Gravity Vector in NED
# --------------------------------
logging.info("Subtask 1.2: Defining gravity vector in NED frame.")

# Gravity vector in NED frame: g_NED = [0, 0, g], where g ≈ 9.81 m/s²
g = 9.81
g_NED = np.array([0.0, 0.0, g])
logging.info(
    f"Gravity vector in NED: {g_NED} m/s^2 (Down positive, magnitude g = {g:.2f} m/s^2)"
)

# --------------------------------
# Subtask 1.3: Define Earth Rotation Rate Vector in NED
# --------------------------------
logging.info("Subtask 1.3: Defining Earth rotation rate vector in NED frame.")

# Earth rotation rate in NED frame: ω_ie,NED = ω_E * [cos(φ), 0, -sin(φ)]
omega_ie_NED = EARTH_RATE * np.array([np.cos(lat), 0.0, -np.sin(lat)])
logging.info(f"Earth rotation rate in NED: {omega_ie_NED} rad/s (North, East, Down)")

# --------------------------------
# Subtask 1.4: Validate and Print Reference Vectors
# --------------------------------
logging.info("Subtask 1.4: Validating reference vectors.")

# Validate vector shapes and components
assert g_NED.shape == (3,), "Gravity vector must be a 3D vector."
assert omega_ie_NED.shape == (3,), "Earth rotation rate vector must be a 3D vector."
assert np.isclose(g_NED[0], 0) and np.isclose(
    g_NED[1], 0
), "Gravity should have no North/East component."
assert np.isclose(
    omega_ie_NED[1], 0
), "Earth rate should have no East component in NED."
logging.info("Reference vectors validated successfully.")

# Print reference vectors
print("==== Reference Vectors in NED Frame ====")
print(f"Gravity vector (NED):        {g_NED} m/s^2")
print(f"Earth rotation rate (NED):   {omega_ie_NED} rad/s")
print(f"Latitude (deg):              {lat_deg:.6f}")
print(f"Longitude (deg):             {lon_deg:.6f}")

# --------------------------------
# Subtask 1.5: Plot Location on Earth Map
# --------------------------------
logging.info("Subtask 1.5: Plotting location on Earth map.")

# Create figure with PlateCarree projection
fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(1, 1, 1, projection=ccrs.PlateCarree())
ax.stock_img()  # Add Earth background image

# Set map extent to focus on the location
ax.set_extent(
    [lon_deg - 5, lon_deg + 5, lat_deg - 5, lat_deg + 5], crs=ccrs.PlateCarree()
)

# Plot the initial location with a red marker
ax.plot(lon_deg, lat_deg, "ro", markersize=10, transform=ccrs.PlateCarree())
ax.text(
    lon_deg + 1,
    lat_deg,
    f"Lat: {lat_deg:.4f}°, Lon: {lon_deg:.4f}°",
    transform=ccrs.PlateCarree(),
)

# Set plot title and save
plt.title("Initial Location on Earth Map")
loc_pdf = OUT / f"{TAG}_task1_location_map.pdf"
plt.savefig(loc_pdf)
if INTERACTIVE:
    plt.show()
plt.close()

logging.info(f"Location map saved as '{loc_pdf}'")


# ================================
# TASK 2: Measure the Vectors in the Body Frame
# ================================
logging.info("TASK 2: Measure the vectors in the body frame")

# --------------------------------
# Subtask 2.1: Load and Parse IMU Data
# --------------------------------
logging.info("Subtask 2.1: Loading and parsing IMU data.")
try:
    data = np.loadtxt("IMU_X001.dat")
except Exception as e:
    logging.error(f"Failed to load IMU data file: {e}")
    raise
if data.shape[1] >= 10:
    acc = data[:, 5:8]  # Velocity increments (m/s)
    gyro = data[:, 2:5]  # Angular increments (rad)
else:
    logging.error("Unexpected data format in IMU_X001.dat.")
    raise ValueError("Invalid IMU data format.")
logging.info(f"IMU data loaded: {data.shape[0]} samples")
print(f"IMU data loaded: {data.shape[0]} samples")

# --------------------------------
# Subtask 2.2: Estimate Static Body-Frame Vectors
# --------------------------------
logging.info(
    "Subtask 2.2: Estimating static body-frame vectors from first 4000 samples."
)
dt_imu = 1.0 / 400.0  # 400 Hz, dt = 0.0025s
acc = acc / dt_imu  # Convert to acceleration (m/s²)
gyro = gyro / dt_imu  # Convert to angular velocity (rad/s)
N_static = min(4000, data.shape[0])
static_acc = np.mean(acc[:N_static], axis=0)
static_gyro = np.mean(gyro[:N_static], axis=0)
logging.info(
    f"Static accelerometer vector (mean over {N_static} samples): {static_acc}"
)
logging.info(f"Static gyroscope vector (mean over {N_static} samples): {static_gyro}")
g_norm = np.linalg.norm(static_acc)
omega_norm = np.linalg.norm(static_gyro)
logging.info(
    f"Estimated gravity magnitude from IMU: {g_norm:.4f} m/s² (expected ~9.81)"
)
logging.info(
    f"Estimated Earth rotation magnitude from IMU: {omega_norm:.6e} rad/s (expected ~{EARTH_RATE})"
)
print(f"Static accelerometer mean: {static_acc}")
print(f"Static gyroscope mean: {static_gyro}")
print(f"Gravity magnitude: {g_norm:.4f} m/s²")
print(f"Earth rotation magnitude: {omega_norm:.6e} rad/s")

# --------------------------------
# Subtask 2.3: Define Gravity and Earth Rate in Body Frame
# --------------------------------
logging.info("Subtask 2.3: Defining gravity and Earth rotation rate in the body frame.")
g_body = -static_acc
omega_ie_body = static_gyro
logging.info(f"Gravity vector in body frame (g_body): {g_body} m/s^2")
logging.info(
    f"Earth rotation rate in body frame (omega_ie_body): {omega_ie_body} rad/s"
)
print(f"Gravity vector (g_body): {g_body} m/s^2")
print(f"Earth rotation rate (omega_ie_body): {omega_ie_body} rad/s")

# --------------------------------
# Subtask 2.4: Validate and Print Body-Frame Vectors
# --------------------------------
logging.info("Subtask 2.4: Validating measured vectors in the body frame.")
expected_omega = EARTH_RATE
assert g_body.shape == (3,), "g_body must be a 3D vector."
assert omega_ie_body.shape == (3,), "omega_ie_body must be a 3D vector."
g_norm = np.linalg.norm(g_body)
omega_norm = np.linalg.norm(omega_ie_body)
if g_norm < 0.1 * 9.81:
    logging.warning(
        "Gravity magnitude is very low; check accelerometer or static assumption."
    )
if omega_norm < 0.5 * expected_omega:
    logging.warning("Earth rotation rate is low; check gyroscope or static assumption.")
logging.info(f"Magnitude of g_body: {g_norm:.6f} m/s^2 (expected ~9.81 m/s^2)")
logging.info(
    f"Magnitude of omega_ie_body: {omega_norm:.6e} rad/s (expected ~{EARTH_RATE:.2e} rad/s)"
)
print("==== Measured Vectors in the Body Frame ====")
print(f"Measured gravity vector (g_body): {g_body} m/s^2")
print(f"Measured Earth rotation (omega_ie_body): {omega_ie_body} rad/s")
print(
    "\nNote: These are the same physical vectors as in NED, but expressed in the body frame (sensor axes)."
)
print("From accelerometer (assuming static IMU):")
print("    a_body = -g_body")
print("From gyroscope:")
print("    ω_ie,body")

# ================================
# TASK 3: Solve Wahba’s Problem
# ================================
import numpy as np
import logging
import pickle
import matplotlib.pyplot as plt

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
    t2_body = (
        np.array([1.0, 0.0, 0.0])
        if abs(v1_B[0]) < abs(v1_B[1])
        else np.array([0.0, 1.0, 0.0])
    )
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
R_tri = (
    np.column_stack((t1_ned, t2_ned, t3_ned))
    @ np.column_stack((t1_body, t2_body, t3_body)).T
)
logging.info("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)
print("Rotation matrix (TRIAD method, Case 1):")
print(R_tri)

# Case 2
t2_ned_temp_doc = np.cross(v1_N, v2_N_doc)
if np.linalg.norm(t2_ned_temp_doc) < 1e-10:
    logging.warning("Case 2 - NED vectors are collinear, TRIAD may fail.")
    t2_ned_doc = np.array([1.0, 0.0, 0.0])
else:
    t2_ned_doc = t2_ned_temp_doc / np.linalg.norm(t2_ned_temp_doc)
t3_ned_doc = np.cross(t1_ned, t2_ned_doc)
R_tri_doc = (
    np.column_stack((t1_ned, t2_ned_doc, t3_ned_doc))
    @ np.column_stack((t1_body, t2_body, t3_body)).T
)
logging.info("Rotation matrix (TRIAD method, Case 2):\n%s", R_tri_doc)
print("Rotation matrix (TRIAD method, Case 2):")
print(R_tri_doc)

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
print("Rotation matrix (Davenport’s Q-Method, Case 1):")
print(R_dav)
print(f"Davenport quaternion (Case 1): {q_dav}")

# Case 2
B_doc = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N_doc, v2_B)
sigma_doc = np.trace(B_doc)
S_doc = B_doc + B_doc.T
Z_doc = np.array(
    [B_doc[1, 2] - B_doc[2, 1], B_doc[2, 0] - B_doc[0, 2], B_doc[0, 1] - B_doc[1, 0]]
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
print("Rotation matrix (Davenport’s Q-Method, Case 2):")
print(R_dav_doc)
print(f"Davenport quaternion (Case 2): {q_dav_doc}")

# --------------------------------
# Subtask 3.4: SVD Method
# --------------------------------
logging.info("Subtask 3.4: Computing rotation matrix using SVD method.")
# Case 1
M = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N, v2_B)
U, _, Vt = np.linalg.svd(M)
R_svd = U @ np.diag([1, 1, np.linalg.det(U) * np.linalg.det(Vt)]) @ Vt
logging.info("Rotation matrix (SVD method, Case 1):\n%s", R_svd)
print("Rotation matrix (SVD method, Case 1):")
print(R_svd)

# Case 2
M_doc = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N_doc, v2_B)
U_doc, _, Vt_doc = np.linalg.svd(M_doc)
R_svd_doc = (
    U_doc @ np.diag([1, 1, np.linalg.det(U_doc) * np.linalg.det(Vt_doc)]) @ Vt_doc
)
logging.info("Rotation matrix (SVD method, Case 2):\n%s", R_svd_doc)
print("Rotation matrix (SVD method, Case 2):")
print(R_svd_doc)

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
logging.info(f"Quaternion (TRIAD, Case 1): {q_tri}")
print(f"Quaternion (TRIAD, Case 1): {q_tri}")
logging.info(f"Quaternion (SVD, Case 1): {q_svd}")
print(f"Quaternion (SVD, Case 1): {q_svd}")
logging.info(f"Quaternion (TRIAD, Case 2): {q_tri_doc}")
print(f"Quaternion (TRIAD, Case 2): {q_tri_doc}")
logging.info(f"Quaternion (SVD, Case 2): {q_svd_doc}")
print(f"Quaternion (SVD, Case 2): {q_svd_doc}")

# --------------------------------
# Subtask 3.6: Validate Attitude Determination and Compare Methods
# --------------------------------
logging.info("Subtask 3.6: Validating attitude determination and comparing methods.")


def angle_error(v_est, v_ref):
    dot = np.clip(np.dot(v_est, v_ref), -1.0, 1.0)
    return np.degrees(np.arccos(dot))


results_x001 = {}
results_x001_doc = {}
methods = {"TRIAD": R_tri}
methods_doc = {"TRIAD": R_tri_doc}

# Case 1: Current implementation
for name, R in methods.items():
    g_est_NED = R @ v1_B
    omega_est_NED = R @ v2_B
    err_g = angle_error(g_est_NED, v1_N)
    err_omega = angle_error(omega_est_NED, v2_N)
    results_x001[name] = {"gravity_error": err_g, "earth_rate_error": err_omega}
    logging.info(
        f"Case X001 - {name} method errors: gravity = {err_g:.4f}°, Earth rate = {err_omega:.4f}°"
    )
    print(
        f"Case X001 - {name} method errors: gravity = {err_g:.4f}°, Earth rate = {err_omega:.4f}°"
    )

# Case 2: Document equations
for name, R in methods_doc.items():
    g_est_NED = R @ v1_B
    omega_est_NED = R @ v2_B
    err_g = angle_error(g_est_NED, v1_N)
    err_omega = angle_error(omega_est_NED, v2_N_doc)
    results_x001_doc[name] = {"gravity_error": err_g, "earth_rate_error": err_omega}
    logging.info(
        f"Case X001_doc - {name} method errors: gravity = {err_g:.4f}°, Earth rate = {err_omega:.4f}°"
    )
    print(
        f"Case X001_doc - {name} method errors: gravity = {err_g:.4f}°, Earth rate = {err_omega:.4f}°"
    )

# Summary table
print("\n==== Method Comparison for Case X001 and Case X001_doc ====")
print(
    f"{'Case':<15} {'Method':<10} {'Gravity Error (deg)':<20} {'Earth Rate Error (deg)':<20}"
)
for name in methods:
    print(
        f"{'X001':<15} {name:<10} {results_x001[name]['gravity_error']:<20.4f} {results_x001[name]['earth_rate_error']:<20.4f}"
    )
for name in methods_doc:
    print(
        f"{'X001_doc':<15} {name:<10} {results_x001_doc[name]['gravity_error']:<20.4f} {results_x001_doc[name]['earth_rate_error']:<20.4f}"
    )

# --------------------------------
# Subtask 3.7: Plot Validation Errors and Quaternion Components
# --------------------------------
import matplotlib.pyplot as plt
import numpy as np
import logging

logging.info("Subtask 3.7: Plotting validation errors and quaternion components.")

# Define methods and cases for plotting (TRIAD only)
methods = ["TRIAD"]
cases = ["Case 1", "Case 2"]

# Collect error data for both cases
# Note: Assumes results_x001 and results_x001_doc are dictionaries containing error data
gravity_errors_case1 = [results_x001[m]["gravity_error"] for m in methods]
earth_rate_errors_case1 = [results_x001[m]["earth_rate_error"] for m in methods]
gravity_errors_case2 = [results_x001_doc[m]["gravity_error"] for m in methods]
earth_rate_errors_case2 = [results_x001_doc[m]["earth_rate_error"] for m in methods]

# Plot error comparison
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
x = np.arange(len(methods))
width = 0.35

# Gravity errors subplot
axes[0].bar(x - width / 2, gravity_errors_case1, width, label="Case 1")
axes[0].bar(x + width / 2, gravity_errors_case2, width, label="Case 2")
axes[0].set_xticks(x)
axes[0].set_xticklabels(methods)
axes[0].set_title("Gravity Error Comparison")
axes[0].set_ylabel("Error (degrees)")
axes[0].legend()

# Earth rate errors subplot
axes[1].bar(x - width / 2, earth_rate_errors_case1, width, label="Case 1")
axes[1].bar(x + width / 2, earth_rate_errors_case2, width, label="Case 2")
axes[1].set_xticks(x)
axes[1].set_xticklabels(methods)
axes[1].set_title("Earth Rate Error Comparison")
axes[1].set_ylabel("Error (degrees)")
axes[1].legend()

plt.tight_layout()
plt.savefig(f"{TAG}_task3_errors_comparison.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(
    f"Error comparison plot saved as '{TAG}_task3_errors_comparison.pdf'"
)

# Collect quaternion data for both cases
# Note: Assumes q_tri, q_dav, q_svd (Case 1) and q_tri_doc, q_dav_doc, q_svd_doc (Case 2) are quaternion arrays
quats_case1 = {"TRIAD": q_tri, "Davenport": q_dav, "SVD": q_svd}
quats_case2 = {"TRIAD": q_tri_doc, "Davenport": q_dav_doc, "SVD": q_svd_doc}

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
ax.legend()

plt.tight_layout()
plt.savefig(f"{TAG}_task3_quaternions_comparison.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(
    f"Quaternion comparison plot saved as '{TAG}_task3_quaternions_comparison.pdf'"
)

# --------------------------------
# Subtask 3.8: Store Rotation Matrices for Later Tasks
# --------------------------------
logging.info("Subtask 3.8: Storing rotation matrices for use in later tasks.")
task3_results = {"TRIAD": {"R": R_tri}, "Davenport": {"R": R_dav}, "SVD": {"R": R_svd}}
logging.info("Task 3 results stored in memory: %s", list(task3_results.keys()))

# ================================
# TASK 4: GNSS and IMU Data Integration and Comparison
# ================================
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import logging
import pickle

# ================================
# TASK 4: GNSS and IMU Data Integration and Comparison
# ================================
logging.info("TASK 4: GNSS and IMU Data Integration and Comparison")

# --------------------------------
# Subtask 4.1: Access Rotation Matrices from Task 3
# --------------------------------
logging.info("Subtask 4.1: Accessing rotation matrices from Task 3.")
methods = ["TRIAD"]
C_B_N_methods = {m: task3_results[m]["R"] for m in methods}
logging.info("Rotation matrices accessed: %s", list(C_B_N_methods.keys()))

# --------------------------------
# Subtask 4.3: Load GNSS Data
# --------------------------------
logging.info("Subtask 4.3: Loading GNSS data.")
try:
    gnss_data = pd.read_csv("GNSS_X001.csv")
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

# --------------------------------
# Subtask 4.5: Define Reference Point
# --------------------------------
logging.info("Subtask 4.5: Defining reference point.")
ref_lat = np.deg2rad(-32.026554)  # From Task 1
ref_lon = np.deg2rad(133.455801)  # From Task 1
ref_r0 = np.array([-3729051, 3935676, -3348394])  # From Task 1
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
gnss_pos_ned = np.array([C_ECEF_to_NED @ (r - ref_r0) for r in gnss_pos_ecef])
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


# --------------------------------
# Subtask 4.9: Load IMU Data and Correct for Bias for Each Method
# --------------------------------
logging.info("Subtask 4.9: Loading IMU data and correcting for bias for each method.")
try:
    imu_data = pd.read_csv("IMU_X001.dat", sep="\s+", header=None)
    imu_time = np.arange(len(imu_data)) * dt_imu

    # Convert velocity increments to acceleration (m/s²)
    # Columns 5,6,7 are velocity increments (m/s) over dt_imu
    acc_body = imu_data[[5, 6, 7]].values / dt_imu  # acc_body = delta_v / dt_imu

    # Convert angular increments to angular rates (rad/s)
    # Columns 2,3,4 are angular increments (rad) over dt_imu
    gyro_body = imu_data[[2, 3, 4]].values / dt_imu  # gyro_body = delta_theta / dt_imu

    N_static = 4000
    if len(imu_data) < N_static:
        raise ValueError("Insufficient static samples for bias estimation.")

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

        # Gyroscope bias: static_gyro should equal C_N_B @ omega_ie_NED
        omega_ie_NED = np.array(
            [
                EARTH_RATE * np.cos(ref_lat),
                0.0,
                -EARTH_RATE * np.sin(ref_lat),
            ]
        )
        omega_ie_body_expected = C_N_B @ omega_ie_NED
        gyro_bias = static_gyro - omega_ie_body_expected  # Bias = measured - expected

        # Correct the entire dataset
        acc_body_corrected[m] = acc_body - acc_bias
        gyro_body_corrected[m] = gyro_body - gyro_bias

        logging.info(f"Method {m}: Accelerometer bias: {acc_bias}")
        logging.info(f"Method {m}: Gyroscope bias: {gyro_bias}")
        print(f"Method {m}: Accelerometer bias: {acc_bias}")
        print(f"Method {m}: Gyroscope bias: {gyro_bias}")

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
pos_integ = {m: np.zeros((len(imu_time), 3)) for m in methods}
vel_integ = {m: np.zeros((len(imu_time), 3)) for m in methods}
acc_integ = {m: np.zeros((len(imu_time), 3)) for m in methods}
logging.info("Output arrays initialized for position, velocity, and acceleration.")

# --------------------------------
# Subtask 4.12: Integrate IMU Accelerations for Each Method
# --------------------------------
logging.info("Subtask 4.12: Integrating IMU accelerations for each method.")
for m in methods:
    logging.info(f"Integrating IMU data using {m} method.")
    C_B_N = C_B_N_methods[m]
    pos = np.zeros(3)
    vel = np.zeros(3)
    for i in range(len(imu_time)):
        if i > 0:
            dt = imu_time[i] - imu_time[i - 1]
            f_ned = (
                C_B_N @ acc_body_corrected[m][i]
            )  # Use method-specific corrected acceleration
            a_ned = f_ned + g_NED
            acc_integ[m][i] = a_ned
            vel += a_ned * dt
            pos += vel * dt
            vel_integ[m][i] = vel
            pos_integ[m][i] = pos
        else:
            acc_integ[m][i] = np.zeros(3)
            vel_integ[m][i] = np.zeros(3)
            pos_integ[m][i] = np.zeros(3)
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
# Validate time ranges
if t_rel_ilu.max() < 1000:
    logging.warning(f"IMU time range too short: {t_rel_ilu.max():.2f} seconds")
if t_rel_gnss.max() < 1000:
    logging.warning(f"GNSS time range too short: {t_rel_gnss.max():.2f} seconds")
print(f"gnss_time range: {gnss_time.min():.2f} to {gnss_time.max():.2f}")
print(f"imu_time range: {imu_time.min():.2f} to {imu_time.max():.2f}")
print(f"t_rel_gnss range: {t_rel_gnss.min():.2f} to {t_rel_gnss.max():.2f}")
print(f"t_rel_ilu range: {t_rel_ilu.min():.2f} to {t_rel_ilu.max():.2f}")

# Comparison plot in NED frame
fig_comp, axes_comp = plt.subplots(3, 3, figsize=(15, 10))
directions = ["North", "East", "Down"]
colors = {"TRIAD": "r", "Davenport": "g", "SVD": "b", "Truth": "m"}
for j in range(3):
    # Position comparison
    ax = axes_comp[0, j]
    ax.plot(t_rel_gnss, gnss_pos_ned[:, j], "k--", label="GNSS Position (direct)")
    for m in methods:
        ax.plot(
            t_rel_ilu,
            pos_integ[m][:, j],
            color=colors[m],
            alpha=0.7,
            label=f"IMU {m} Position (derived)",
        )
    ax.set_title(f"Position {directions[j]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.legend()

    # Velocity comparison
    ax = axes_comp[1, j]
    ax.plot(t_rel_gnss, gnss_vel_ned[:, j], "k--", label="GNSS Velocity (direct)")
    for m in methods:
        ax.plot(
            t_rel_ilu,
            vel_integ[m][:, j],
            color=colors[m],
            alpha=0.7,
            label=f"IMU {m} Velocity (derived)",
        )
    ax.set_title(f"Velocity {directions[j]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.legend()

    # Acceleration comparison
    ax = axes_comp[2, j]
    ax.plot(t_rel_gnss, gnss_acc_ned[:, j], "k--", label="GNSS Derived")
    for m in methods:
        ax.plot(
            t_rel_ilu,
            acc_integ[m][:, j],
            color=colors[m],
            alpha=0.7,
            label=f"IMU {m} Acceleration",
        )
    ax.set_title(f"Acceleration {directions[j]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.legend()
plt.tight_layout()
plt.savefig(f"{TAG}_task4_comparison_ned.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(
    f"Comparison plot in NED frame saved as '{TAG}_task4_comparison_ned.pdf'"
)

# Plot 1: Data in mixed frames (GNSS position/velocity in ECEF, IMU acceleration in body)
logging.info("Plotting data in mixed frames.")
fig_mixed, axes_mixed = plt.subplots(3, 3, figsize=(15, 10))
directions_pos = ["X_ECEF", "Y_ECEF", "Z_ECEF"]
directions_vel = ["VX_ECEF", "VY_ECEF", "VZ_ECEF"]
directions_acc = ["AX_body", "AY_body", "AZ_body"]
for i in range(3):
    for j in range(3):
        ax = axes_mixed[i, j]
        if i == 0:  # Position
            ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], "k-", label="GNSS Position (ECEF)")
            ax.set_title(f"Position {directions_pos[j]}")
        elif i == 1:  # Velocity
            ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], "k-", label="GNSS Velocity (ECEF)")
            ax.set_title(f"Velocity {directions_vel[j]}")
        else:  # Acceleration
            for m in methods:
                ax.plot(
                    t_rel_ilu,
                    acc_body_corrected[m][:, j],
                    color=colors[m],
                    alpha=0.7,
                    label=f"IMU {m} Acceleration (Body)",
                )
            ax.set_title(f"Acceleration {directions_acc[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.legend()
plt.tight_layout()
plt.savefig(f"{TAG}_task4_mixed_frames.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(f"Mixed frames plot saved as '{TAG}_task4_mixed_frames.pdf'")

# Plot 2: All data in NED frame
logging.info("Plotting all data in NED frame.")
fig_ned, axes_ned = plt.subplots(3, 3, figsize=(15, 10))
directions_ned = ["N", "E", "D"]
for i in range(3):
    for j in range(3):
        ax = axes_ned[i, j]
        if i == 0:  # Position
            ax.plot(t_rel_gnss, gnss_pos_ned[:, j], "k-", label="GNSS Position (NED)")
            ax.set_title(f"Position {directions_ned[j]}")
        elif i == 1:  # Velocity
            ax.plot(t_rel_gnss, gnss_vel_ned[:, j], "k-", label="GNSS Velocity (NED)")
            ax.set_title(f"Velocity V{directions_ned[j]}")
        else:  # Acceleration
            for m in methods:
                f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                ax.plot(
                    t_rel_ilu,
                    f_ned[j],
                    color=colors[m],
                    alpha=0.7,
                    label=f"IMU {m} Acceleration (NED)",
                )
            ax.set_title(f"Acceleration A{directions_ned[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.legend()
plt.tight_layout()
plt.savefig(f"{TAG}_task4_all_ned.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(f"All data in NED frame plot saved as '{TAG}_task4_all_ned.pdf'")

# Plot 3: All data in ECEF frame
logging.info("Plotting all data in ECEF frame.")
fig_ecef, axes_ecef = plt.subplots(3, 3, figsize=(15, 10))
directions_ecef = ["X", "Y", "Z"]
for i in range(3):
    for j in range(3):
        ax = axes_ecef[i, j]
        if i == 0:  # Position
            ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], "k-", label="GNSS Position (ECEF)")
            ax.set_title(f"Position {directions_ecef[j]}_ECEF")
        elif i == 1:  # Velocity
            ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], "k-", label="GNSS Velocity (ECEF)")
            ax.set_title(f"Velocity V{directions_ecef[j]}_ECEF")
        else:  # Acceleration
            for m in methods:
                f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                f_ecef = C_NED_to_ECEF @ f_ned
                ax.plot(
                    t_rel_ilu,
                    f_ecef[j],
                    color=colors[m],
                    alpha=0.7,
                    label=f"IMU {m} Acceleration (ECEF)",
                )
            ax.set_title(f"Acceleration A{directions_ecef[j]}_ECEF")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.legend()
plt.tight_layout()
plt.savefig(f"{TAG}_task4_all_ecef.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(f"All data in ECEF frame plot saved as '{TAG}_task4_all_ecef.pdf'")

# Plot 4: All data in body frame
logging.info("Plotting all data in body frame.")
fig_body, axes_body = plt.subplots(3, 3, figsize=(15, 10))
directions_body = ["X", "Y", "Z"]
for i in range(3):
    for j in range(3):
        ax = axes_body[i, j]
        if i == 0:  # Position
            r_body = (C_N_B @ gnss_pos_ned.T).T
            ax.plot(t_rel_gnss, r_body[:, j], "k-", label="Position in body frame")
            ax.set_title(f"Position r{directions_body[j]}_body")
        elif i == 1:  # Velocity
            vel_body = (C_N_B @ gnss_vel_ned.T).T
            ax.plot(t_rel_gnss, vel_body[:, j], "k-", label="Velocity in body frame")
            ax.set_title(f"Velocity v{directions_body[j]}_body")
        else:  # Acceleration
            for m in methods:
                ax.plot(
                    t_rel_ilu,
                    acc_body_corrected[m][:, j],
                    color=colors[m],
                    alpha=0.7,
                    label=f"IMU {m} Acceleration (Body)",
                )
            ax.set_title(f"Acceleration A{directions_body[j]}_body")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.legend()
plt.tight_layout()
plt.savefig(f"{TAG}_task4_all_body.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(f"All data in body frame plot saved as '{TAG}_task4_all_body.pdf'")


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import logging
from filterpy.kalman import KalmanFilter
import pickle

# ================================
# Task 5: Sensor Fusion with Kalman Filter
# ================================
logging.info("Task 5: Sensor Fusion with Kalman Filter")

# --------------------------------
# Subtask 5.1: Configure Logging
# --------------------------------
logging.info("Subtask 5.1: Configuring logging.")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

# --------------------------------
# Subtask 5.2: Rotation Matrix - ECEF to NED
# --------------------------------
# Subtask 5.3: Load GNSS and IMU Data
# --------------------------------
logging.info("Subtask 5.3: Loading GNSS and IMU data.")
try:
    gnss_data = pd.read_csv("GNSS_X001.csv")
    imu_data = pd.read_csv("IMU_X001.dat", sep="\s+", header=None)
except Exception as e:
    logging.error(f"Failed to load data: {e}")
    raise

# Extract GNSS fields
time_col = "Posix_Time"
pos_cols = ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]
vel_cols = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
gnss_time = gnss_data[time_col].values
gnss_pos_ecef = gnss_data[pos_cols].values
gnss_vel_ecef = gnss_data[vel_cols].values

# Reference position for NED
ref_lat = np.deg2rad(-32.026554)  # From Task 1
ref_lon = np.deg2rad(133.455801)  # From Task 1
ref_r0 = np.array([-3729051, 3935676, -3348394])  # From Task 1
C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon)

# Convert GNSS to NED
gnss_pos_ned = np.array([C_ECEF_to_NED @ (r - ref_r0) for r in gnss_pos_ecef])
gnss_vel_ned = np.array([C_ECEF_to_NED @ v for v in gnss_vel_ecef])

# Compute GNSS acceleration
gnss_acc_ecef = np.zeros_like(gnss_vel_ecef)
dt = np.diff(gnss_time, prepend=gnss_time[0])
gnss_acc_ecef[1:] = (gnss_vel_ecef[1:] - gnss_vel_ecef[:-1]) / dt[1:, np.newaxis]
gnss_acc_ned = np.array([C_ECEF_to_NED @ a for a in gnss_acc_ecef])

# Load IMU data
imu_time = np.arange(len(imu_data)) * dt_imu
acc_body = imu_data[[5, 6, 7]].values / dt_imu
N_static = 4000
if len(imu_data) < N_static:
    raise ValueError("Insufficient static samples.")
static_acc = np.mean(acc_body[:N_static], axis=0)


# Compute corrected acceleration for each method
acc_body_corrected = {}
for m in methods:
    C_N_B = C_B_N_methods[m].T
    g_body_expected = C_N_B @ g_NED
    bias = static_acc + g_body_expected
    acc_body_corrected[m] = acc_body - bias
    logging.info(f"Method {m}: Bias computed: {bias}")

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

# --------------------------------
# Subtask 5.5: Apply Zero-Velocity Updates (ZUPT) for Each Method
# --------------------------------
logging.info("Subtask 5.5: Applying Zero-Velocity Updates (ZUPT) for each method.")
stationary_threshold = 0.01
t_rel_ilu = imu_time
for m in methods:
    for i in range(len(imu_time)):
        if t_rel_ilu[i] < 5000 and np.linalg.norm(imu_vel[m][i]) < stationary_threshold:
            imu_vel[m][i] = np.zeros(3)
            imu_pos[m][i] = imu_pos[m][i - 1]
    logging.info(f"Method {m}: ZUPT applied.")

# --------------------------------
# Subtask 5.6: Kalman Filter for Sensor Fusion for Each Method
# --------------------------------
logging.info("Subtask 5.6: Running Kalman Filter for sensor fusion for each method.")
fused_pos = {m: np.zeros_like(imu_pos[m]) for m in methods}
fused_vel = {m: np.zeros_like(imu_vel[m]) for m in methods}
fused_acc = {m: np.zeros_like(imu_acc[m]) for m in methods}
for m in methods:
    kf = KalmanFilter(dim_x=9, dim_z=6)
    kf.x = np.hstack((imu_pos[m][0], imu_vel[m][0], imu_acc[m][0]))
    kf.F = np.eye(9)
    kf.H = np.hstack((np.eye(6), np.zeros((6, 3))))
    kf.P *= 1.0
    kf.R = np.eye(6) * 0.1
    kf.Q = np.eye(9) * 0.01
    fused_pos[m][0] = imu_pos[m][0]
    fused_vel[m][0] = imu_vel[m][0]
    fused_acc[m][0] = imu_acc[m][0]

    # Interpolate GNSS data to IMU time
    gnss_pos_ned_interp = np.zeros_like(imu_pos[m])
    gnss_vel_ned_interp = np.zeros_like(imu_vel[m])
    for j in range(3):
        gnss_pos_ned_interp[:, j] = np.interp(imu_time, gnss_time, gnss_pos_ned[:, j])
        gnss_vel_ned_interp[:, j] = np.interp(imu_time, gnss_time, gnss_vel_ned[:, j])

    # Run Kalman Filter
    for i in range(1, len(imu_time)):
        dt = imu_time[i] - imu_time[i - 1]
        # Prediction step
        kf.F[0:3, 3:6] = np.eye(3) * dt
        kf.F[3:6, 6:9] = np.eye(3) * dt
        kf.predict()
        # Update step
        z = np.hstack((gnss_pos_ned_interp[i], gnss_vel_ned_interp[i]))
        kf.update(z)
        fused_pos[m][i] = kf.x[0:3]
        fused_vel[m][i] = kf.x[3:6]
        fused_acc[m][i] = imu_acc[m][i]  # Use integrated acceleration
    logging.info(f"Method {m}: Kalman Filter completed.")

# --------------------------------
# Subtask 5.7: Handle Event at 5000s (if needed)
# --------------------------------
logging.info("Subtask 5.7: No event handling needed as time < 5000s.")

# --------------------------------
# Subtask 5.8: Plotting Results for All Methods
# --------------------------------

import matplotlib.pyplot as plt
import numpy as np
import logging

# Configure logging if not already done
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

# Define methods and colors (TRIAD only)
methods = ["TRIAD"]
colors = {"TRIAD": "r", "Truth": "m"}  # Red for TRIAD, magenta for truth
directions = ["North", "East", "Down"]

# Interpolate GNSS acceleration to IMU time (done once for all plots)
gnss_acc_ned_interp = np.zeros((len(imu_time), 3))
for j in range(3):
    gnss_acc_ned_interp[:, j] = np.interp(imu_time, gnss_time, gnss_acc_ned[:, j])
    logging.info(
        f"Interpolated GNSS acceleration for {directions[j]} direction: "
        f"First value = {gnss_acc_ned_interp[0, j]:.4f}, "
        f"Last value = {gnss_acc_ned_interp[-1, j]:.4f}"
    )
    print(
        f"# Interpolated GNSS acceleration {directions[j]}: "
        f"First = {gnss_acc_ned_interp[0, j]:.4f}, Last = {gnss_acc_ned_interp[-1, j]:.4f}"
    )

# Attempt to load ground truth trajectory in STATE_X001.txt
try:
    truth = np.loadtxt("STATE_X001.txt")
    truth_time = truth[:, 1]
    truth_pos_ecef = truth[:, 2:5]
    truth_vel_ecef = truth[:, 5:8]
    truth_pos_ned = np.array([C_ECEF_to_NED @ (p - ref_r0) for p in truth_pos_ecef])
    truth_vel_ned = np.array([C_ECEF_to_NED @ v for v in truth_vel_ecef])
    truth_acc_ned = np.zeros_like(truth_vel_ned)
    dt_truth = np.diff(truth_time, prepend=truth_time[0])
    truth_acc_ned[1:] = (truth_vel_ned[1:] - truth_vel_ned[:-1]) / dt_truth[
        1:, np.newaxis
    ]
    truth_pos_ned_interp = np.vstack(
        [np.interp(imu_time, truth_time, truth_pos_ned[:, i]) for i in range(3)]
    ).T
    truth_vel_ned_interp = np.vstack(
        [np.interp(imu_time, truth_time, truth_vel_ned[:, i]) for i in range(3)]
    ).T
    truth_acc_ned_interp = np.vstack(
        [np.interp(imu_time, truth_time, truth_acc_ned[:, i]) for i in range(3)]
    ).T
    logging.info("Loaded ground truth trajectory.")
except Exception as e:
    truth_pos_ned_interp = truth_vel_ned_interp = truth_acc_ned_interp = None
    logging.warning(f"Could not load truth data: {e}")

# Subtask 5.8.1: Plotting Results for TRIAD
logging.info("Subtask 5.8.1: Plotting results for TRIAD.")
print("# Subtask 5.8.1: Starting to plot results for TRIAD.")
fig, axes = plt.subplots(3, 3, figsize=(15, 10))

# TRIAD - Position
for j in range(3):
    ax = axes[0, j]
    ax.plot(imu_time, gnss_pos_ned_interp[:, j], "k-", label="GNSS")
    if truth_pos_ned_interp is not None:
        ax.plot(
            imu_time,
            truth_pos_ned_interp[:, j],
            colors["Truth"],
            linestyle="--",
            label="Truth",
        )
    ax.plot(
        imu_time,
        fused_pos["TRIAD"][:, j],
        colors["TRIAD"],
        alpha=0.7,
        label="Fused TRIAD",
    )
    ax.set_title(f"Position {directions[j]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.legend()
    logging.info(
        f"Subtask 5.8.1: Plotted TRIAD position {directions[j]}: "
        f"First = {fused_pos['TRIAD'][0, j]:.4f}, Last = {fused_pos['TRIAD'][-1, j]:.4f}"
    )
    print(
        f"# Plotted TRIAD position {directions[j]}: "
        f"First = {fused_pos['TRIAD'][0, j]:.4f}, Last = {fused_pos['TRIAD'][-1, j]:.4f}"
    )

# TRIAD - Velocity
for j in range(3):
    ax = axes[1, j]
    ax.plot(imu_time, gnss_vel_ned_interp[:, j], "k-", label="GNSS")
    if truth_vel_ned_interp is not None:
        ax.plot(
            imu_time,
            truth_vel_ned_interp[:, j],
            colors["Truth"],
            linestyle="--",
            label="Truth",
        )
    ax.plot(
        imu_time,
        fused_vel["TRIAD"][:, j],
        colors["TRIAD"],
        alpha=0.7,
        label="Fused TRIAD",
    )
    ax.set_title(f"Velocity {directions[j]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.legend()
    logging.info(
        f"Subtask 5.8.1: Plotted TRIAD velocity {directions[j]}: "
        f"First = {fused_vel['TRIAD'][0, j]:.4f}, Last = {fused_vel['TRIAD'][-1, j]:.4f}"
    )
    print(
        f"# Plotted TRIAD velocity {directions[j]}: "
        f"First = {fused_vel['TRIAD'][0, j]:.4f}, Last = {fused_vel['TRIAD'][-1, j]:.4f}"
    )

# TRIAD - Acceleration
for j in range(3):
    ax = axes[2, j]
    ax.plot(imu_time, gnss_acc_ned_interp[:, j], "k-", label="GNSS")
    if truth_acc_ned_interp is not None:
        ax.plot(
            imu_time,
            truth_acc_ned_interp[:, j],
            colors["Truth"],
            linestyle="--",
            label="Truth",
        )
    ax.plot(
        imu_time,
        fused_acc["TRIAD"][:, j],
        colors["TRIAD"],
        alpha=0.7,
        label="Fused TRIAD",
    )
    ax.set_title(f"Acceleration {directions[j]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.legend()
    logging.info(
        f"Subtask 5.8.1: Plotted TRIAD acceleration {directions[j]}: "
        f"First = {fused_acc['TRIAD'][0, j]:.4f}, Last = {fused_acc['TRIAD'][-1, j]:.4f}"
    )
    print(
        f"# Plotted TRIAD acceleration {directions[j]}: "
        f"First = {fused_acc['TRIAD'][0, j]:.4f}, Last = {fused_acc['TRIAD'][-1, j]:.4f}"
    )

plt.tight_layout()
plt.savefig(f"{TAG}_task5_results_TRIAD.pdf")
if INTERACTIVE:
    plt.show()
plt.close()
logging.info(
    f"Subtask 5.8.1: TRIAD plot saved as '{TAG}_task5_results_TRIAD.pdf'"
)
print("# Subtask 5.8.1: TRIAD plotting completed.")

# Colors: TRIAD (red), Davenport (green), SVD (blue), GNSS (black), Truth (magenta).

# ================================
# Task 6: Plot residuals and attitude angles
# ================================
logging.info("Task 6: plotting residuals and attitude angles.")

from scipy.spatial.transform import Rotation as R

# Choose the method to analyze (use the first method by default)
method = methods[0] if isinstance(methods, list) else "TRIAD"

# Interpolate fused data to GNSS time for residual computation
fused_pos_interp = np.vstack(
    [np.interp(gnss_time, imu_time, fused_pos[method][:, i]) for i in range(3)]
).T
fused_vel_interp = np.vstack(
    [np.interp(gnss_time, imu_time, fused_vel[method][:, i]) for i in range(3)]
).T

# Compute residuals
residual_pos = fused_pos_interp - gnss_pos_ned
residual_vel = fused_vel_interp - gnss_vel_ned

labels = ["North", "East", "Down"]
fig, axes = plt.subplots(2, 1, figsize=(10, 8))
for i, lbl in enumerate(labels):
    axes[0].plot(gnss_time, residual_pos[:, i], label=lbl)
axes[0].set_xlabel("Time (s)")
axes[0].set_ylabel("Position Residual (m)")
axes[0].set_title("Position Residuals vs. Time")
axes[0].legend(loc="best")

for i, lbl in enumerate(labels):
    axes[1].plot(gnss_time, residual_vel[:, i], label=lbl)
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Velocity Residual (m/s)")
axes[1].set_title("Velocity Residuals vs. Time")
axes[1].legend(loc="best")

plt.tight_layout()
plt.savefig(f"{TAG}_residuals.pdf")
plt.close()

# Compute attitude angles (roll, pitch, yaw) for the selected method
R_BN = C_B_N_methods[method]
euler_static = R.from_matrix(R_BN).as_euler("xyz", degrees=True)
euler_angles = np.tile(euler_static, (len(imu_time), 1))

plt.figure()
plt.plot(imu_time, euler_angles[:, 0], label="Roll")
plt.plot(imu_time, euler_angles[:, 1], label="Pitch")
plt.plot(imu_time, euler_angles[:, 2], label="Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Attitude Angles vs. Time")
plt.legend(loc="best")
plt.tight_layout()
plt.savefig(f"{TAG}_attitude_angles.pdf")
plt.close()
