"""TRIAD-based IMU/GNSS processing demonstration.

This script consolidates the step-by-step task code used during
project development into a single file.  It performs the following:

1. Compute reference vectors (gravity and Earth rate) from GNSS ECEF
   coordinates.
2. Measure the same vectors in the body frame using IMU data.
3. Solve Wahba's problem using the TRIAD method to
   determine the initial attitude.
4. Compare GNSS and IMU data in several coordinate frames and generate
   diagnostic plots.
5. Run a basic Kalman filter to fuse GNSS and IMU measurements.

Running the script will produce a collection of PDF figures in the
working directory summarising each stage.  Example usage::

    python workflow.py

The script expects the GNSS and IMU example data files distributed with
this repository (``GNSS_X001.csv``, ``IMU_X001.dat`` etc.) to be
available in the current directory.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cartopy.crs as ccrs
import logging
from filterpy.kalman import KalmanFilter

# ---------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------

def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert ECEF coordinates to WGS-84 geodetic latitude, longitude, altitude."""
    a = 6378137.0
    e_sq = 6.69437999014e-3
    p = np.sqrt(x**2 + y**2)
    theta = np.arctan2(z * a, p * (1 - e_sq))
    lon = np.arctan2(y, x)
    lat = np.arctan2(z + e_sq * a * np.sin(theta) ** 3 / (1 - e_sq),
                     p - e_sq * a * np.cos(theta) ** 3)
    N = a / np.sqrt(1 - e_sq * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), alt


def compute_C_ECEF_to_NED(lat: float, lon: float) -> np.ndarray:
    """Rotation matrix from ECEF to local NED frame."""
    sin_phi = np.sin(lat)
    cos_phi = np.cos(lat)
    sin_lambda = np.sin(lon)
    cos_lambda = np.cos(lon)
    return np.array([
        [-sin_phi * cos_lambda, -sin_phi * sin_lambda, cos_phi],
        [-sin_lambda, cos_lambda, 0.0],
        [-cos_phi * cos_lambda, -cos_phi * sin_lambda, -sin_phi],
    ])


def rot_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix ``R`` to ``[qw, qx, qy, qz]`` quaternion."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
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


# ---------------------------------------------------------------------
# Task 1: reference vectors from GNSS
# ---------------------------------------------------------------------

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s",
                    datefmt="%Y-%m-%d %H:%M:%S")
logging.info("TASK 1: Define reference vectors in NED frame")

try:
    gnss_data = pd.read_csv("GNSS_X002.csv")
except Exception as exc:
    logging.error("Failed to load GNSS data file: %s", exc)
    raise

valid_rows = gnss_data[(gnss_data["X_ECEF_m"] != 0) |
                       (gnss_data["Y_ECEF_m"] != 0) |
                       (gnss_data["Z_ECEF_m"] != 0)]
if valid_rows.empty:
    raise ValueError("No valid ECEF coordinates found in GNSS data")

row0 = valid_rows.iloc[0]
lat_deg, lon_deg, _alt = ecef_to_geodetic(float(row0["X_ECEF_m"]),
                                          float(row0["Y_ECEF_m"]),
                                          float(row0["Z_ECEF_m"]))
lat = np.deg2rad(lat_deg)
lon = np.deg2rad(lon_deg)

logging.info("Initial location: %.6f deg %.6f deg", lat_deg, lon_deg)

g_NED = np.array([0.0, 0.0, 9.81])
omega_E = 7.2921159e-5
omega_ie_NED = omega_E * np.array([np.cos(lat), 0.0, -np.sin(lat)])

print("==== Reference Vectors in NED Frame ====")
print("Gravity vector (NED):", g_NED)
print("Earth rotation rate (NED):", omega_ie_NED)
print(f"Latitude: {lat_deg:.4f} deg, Longitude: {lon_deg:.4f} deg")

# Small map showing the point
fig = plt.figure(figsize=(8, 4))
ax = fig.add_subplot(1, 1, 1, projection=ccrs.PlateCarree())
ax.stock_img()
ax.set_extent([lon_deg - 5, lon_deg + 5, lat_deg - 5, lat_deg + 5], crs=ccrs.PlateCarree())
ax.plot(lon_deg, lat_deg, "ro", markersize=8, transform=ccrs.PlateCarree())
ax.text(lon_deg + 1, lat_deg, f"Lat {lat_deg:.2f}\nLon {lon_deg:.2f}", transform=ccrs.PlateCarree())
plt.title("Initial Location")
plt.savefig("1_Initial_Location_Map.pdf")
plt.close()

# ---------------------------------------------------------------------
# Task 2: measure vectors in body frame
# ---------------------------------------------------------------------

logging.info("TASK 2: Measure the vectors in the body frame")
try:
    imu = np.loadtxt("IMU_X002.dat")
except Exception as exc:
    logging.error("Failed to load IMU data file: %s", exc)
    raise

acc = imu[:, 5:8]
gyro = imu[:, 2:5]
dt_imu = 1.0 / 400.0
acc = acc / dt_imu
gyro = gyro / dt_imu
N_static = min(4000, len(acc))
static_acc = np.mean(acc[:N_static], axis=0)
static_gyro = np.mean(gyro[:N_static], axis=0)

g_body = -static_acc
omega_ie_body = static_gyro

print("==== Measured Vectors in Body Frame ====")
print("Gravity vector (body):", g_body)
print("Earth rotation (body):", omega_ie_body)

# ---------------------------------------------------------------------
# Task 3: Wahba's problem
# ---------------------------------------------------------------------

logging.info("TASK 3: Solve Wahba's problem")

v1_B = g_body / np.linalg.norm(g_body)
v2_B = omega_ie_body / np.linalg.norm(omega_ie_body)
v1_N = g_NED / np.linalg.norm(g_NED)
v2_N = omega_ie_NED / np.linalg.norm(omega_ie_NED)

# --- TRIAD ---
t1_b = v1_B
t2_b = np.cross(v1_B, v2_B)
t2_b /= np.linalg.norm(t2_b)
t3_b = np.cross(t1_b, t2_b)

t1_n = v1_N
t2_n = np.cross(v1_N, v2_N)
t2_n /= np.linalg.norm(t2_n)
t3_n = np.cross(t1_n, t2_n)

R_tri = np.column_stack((t1_n, t2_n, t3_n)) @ np.column_stack((t1_b, t2_b, t3_b)).T
q_tri = rot_to_quaternion(R_tri)
print("\nRotation quaternion (body to NED) using TRIAD:")
print(q_tri)


# ---------------------------------------------------------------------
# Task 4: GNSS and IMU integration
# ---------------------------------------------------------------------

logging.info("TASK 4: GNSS and IMU Data Integration")

gnss_data = pd.read_csv("GNSS_X001.csv")
imu_data = pd.read_csv("IMU_X001.dat", sep="\s+", header=None)

C_ECEF_to_NED = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
C_NED_to_ECEF = C_ECEF_to_NED.T

# GNSS to NED
pos_ecef = gnss_data[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].values
vel_ecef = gnss_data[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].values
pos_ned = np.array([C_ECEF_to_NED @ (r - pos_ecef[0]) for r in pos_ecef])
vel_ned = np.array([C_ECEF_to_NED @ v for v in vel_ecef])

# IMU bias correction
imu_time = np.arange(len(imu_data)) * dt_imu + gnss_data["Posix_Time"].values[0]
acc_body = imu_data[[5,6,7]].values / dt_imu
gyro_body = imu_data[[2,3,4]].values / dt_imu
static_acc = np.mean(acc_body[:N_static], axis=0)
static_gyro = np.mean(gyro_body[:N_static], axis=0)

bias_acc = static_acc + (R_tri.T @ g_NED)
bias_gyro = static_gyro - (R_tri.T @ omega_ie_NED)
acc_body -= bias_acc
gyro_body -= bias_gyro

# Integrate specific force
vel_est = np.zeros((len(acc_body), 3))
pos_est = np.zeros((len(acc_body), 3))
for i in range(1, len(acc_body)):
    dt = dt_imu
    f_ned = R_tri @ acc_body[i]
    a_ned = f_ned + g_NED
    vel_est[i] = vel_est[i-1] + a_ned * dt
    pos_est[i] = pos_est[i-1] + vel_est[i] * dt

# ---------------------------------------------------------------------
# Task 5: simple Kalman filter
# ---------------------------------------------------------------------

logging.info("TASK 5: Sensor Fusion with Kalman Filter")

time_gnss = gnss_data["Posix_Time"].values
z = np.hstack((pos_ned, vel_ned))

kf = KalmanFilter(dim_x=6, dim_z=6)
kf.x = z[0]
kf.P = np.eye(6)
kf.H = np.eye(6)
kf.Q = np.eye(6) * 0.01
kf.R = np.eye(6) * 0.1

xs = [kf.x.copy()]
residuals = [np.zeros(6)]
next_idx = 1
for i in range(1, len(imu_time)):
    dt = dt_imu
    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    kf.F = F
    kf.predict()
    t_abs = imu_time[i]
    while next_idx < len(time_gnss) and t_abs >= time_gnss[next_idx]:
        residual = z[next_idx] - (kf.H @ kf.x)
        kf.update(z[next_idx])
        xs.append(kf.x.copy())
        residuals.append(residual)
        next_idx += 1

xs = np.array(xs)
residuals = np.array(residuals)

# Plot results
plt.figure()
for j, lab in enumerate("NED"):
    plt.plot(time_gnss[:len(xs)] - time_gnss[0], xs[:, j], label=f"KF {lab}")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.tight_layout()
plt.savefig("5_Fusion_Positions.pdf")
plt.close()

plt.figure()
for j, lab in enumerate(["North", "East", "Down"]):
    plt.plot(time_gnss[:len(residuals)] - time_gnss[0], residuals[:, j], label=f"pos {lab}")
plt.xlabel("Time (s)")
plt.ylabel("Residual")
plt.legend()
plt.tight_layout()
plt.savefig("5_Fusion_Residuals.pdf")
plt.close()

# ---------------------------------------------------------------------
# Additional evaluation: residual statistics and attitude estimation
# ---------------------------------------------------------------------

t_rel_gnss = gnss_data["Posix_Time"].values - gnss_data["Posix_Time"].values[0]
t_rel_imu = imu_time - imu_time[0]

# Interpolate GNSS data to IMU timestamps for residual analysis
gnss_pos_ned_interp = np.zeros((len(imu_time), 3))
gnss_vel_ned_interp = np.zeros((len(imu_time), 3))
for j in range(3):
    gnss_pos_ned_interp[:, j] = np.interp(imu_time, gnss_data["Posix_Time"].values, pos_ned[:, j])
    gnss_vel_ned_interp[:, j] = np.interp(imu_time, gnss_data["Posix_Time"].values, vel_ned[:, j])

# Residuals between fused and GNSS measurements
fused_pos = xs[:, 0:3]
fused_vel = xs[:, 3:6]
gnss_pos_ned_interp = gnss_pos_ned_interp[: len(fused_pos)]
gnss_vel_ned_interp = gnss_vel_ned_interp[: len(fused_vel)]
pos_residual = fused_pos - gnss_pos_ned_interp
vel_residual = fused_vel - gnss_vel_ned_interp

fig, axes = plt.subplots(2, 1, figsize=(10, 8))
for j, lab in enumerate(["North", "East", "Down"]):
    axes[0].plot(t_rel_gnss[: len(pos_residual)], pos_residual[:, j], label=lab)
axes[0].set_title("Position Residuals")
axes[0].set_xlabel("Time (s)")
axes[0].set_ylabel("ΔPosition (m)")
axes[0].legend()

for j, lab in enumerate(["North", "East", "Down"]):
    axes[1].plot(t_rel_gnss[: len(vel_residual)], vel_residual[:, j], label=lab)
axes[1].set_title("Velocity Residuals")
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("ΔVelocity (m/s)")
axes[1].legend()

plt.tight_layout()
plt.savefig("5_Filter_Residuals_TRIAD.pdf")
plt.close()


def small_angle_quat(omega: np.ndarray, dt: float) -> np.ndarray:
    """Quaternion from small-angle approximation."""
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    q = np.hstack([np.cos(theta / 2), axis * np.sin(theta / 2)])
    return q / np.linalg.norm(q)


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t2 = 2 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)
    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return np.degrees([roll, pitch, yaw])


# Integrate gyroscope data to estimate attitude over time
qts = np.zeros((len(imu_time), 4))
R0 = R_tri
qw = np.sqrt(1 + np.trace(R0)) / 2
qx = (R0[2, 1] - R0[1, 2]) / (4 * qw)
qy = (R0[0, 2] - R0[2, 0]) / (4 * qw)
qz = (R0[1, 0] - R0[0, 1]) / (4 * qw)
qts[0] = [qw, qx, qy, qz]
for k in range(1, len(imu_time)):
    dq = small_angle_quat(gyro_body[k], dt_imu)
    qts[k] = quat_mult(qts[k - 1], dq)

euler = np.array([quat_to_euler(q) for q in qts])

fig, ax = plt.subplots(3, 1, figsize=(10, 8))
for idx, name in enumerate(["Roll", "Pitch", "Yaw"]):
    ax[idx].plot(t_rel_imu, euler[:, idx], label=name)
    ax[idx].set_ylabel(f"{name} (°)")
    ax[idx].legend()
ax[2].set_xlabel("Time (s)")
fig.suptitle("Task 5: TRIAD Attitude Angles Over Time")
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig("5_Attitude_TRIAD.pdf")
plt.close()

