"""IMU/GNSS Fusion Demo (standalone)

This script replicates the small ``imu_fusion`` package provided in this
repository in a single file so it can be run easily inside Spyder or from

the command line. It loads GNSS and IMU data, initialises a simple Kalman
filter and produces PDF plots summarising the results.

Steps:
1. Load GNSS measurements from a CSV file.
2. Load IMU measurements from a whitespace separated ``.dat`` file.
3. Compute an initial attitude quaternion from the first GNSS latitude and
   longitude entry.
4. Run a constant–velocity Kalman filter that fuses GNSS position and
   velocity data.
5. Output ``positions.pdf``, ``residuals.pdf`` and ``attitude.pdf``
   containing the estimated trajectory, filter residuals and basic attitude
   angles.

Example::

    python imu_gnss_fusion_single.py --gnss-file GNSS_X001.csv --imu-file IMU_X001.dat
    python imu_gnss_fusion_single.py --gnss-file GNSS_X002.csv --imu-file IMU_X002.dat

See ``README.md`` for a description of the example data files.  The plots
generated match those listed in ``plot_summary.md``.
"""

from __future__ import annotations

import argparse
from typing import Iterable, Sequence, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------

def load_gnss_csv(path: str) -> pd.DataFrame:
    """Load GNSS data from ``path`` using :func:`pandas.read_csv`."""
    return pd.read_csv(path)


def load_imu_dat(path: str) -> np.ndarray:
    """Load IMU data from a whitespace separated ``.dat`` file."""
    return np.loadtxt(path)


# ---------------------------------------------------------------------------
# Attitude utilities
# ---------------------------------------------------------------------------

def compute_C_ECEF_to_NED(lat: float, lon: float) -> np.ndarray:
    """Rotation matrix from ECEF to local NED for ``lat`` and ``lon``."""
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


def quat_multiply(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Quaternion multiplication ``q * r``."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion ``q`` to unit length."""
    return q / np.linalg.norm(q)


# ---------------------------------------------------------------------------
# Kalman filter utilities
# ---------------------------------------------------------------------------

def simple_kalman(
    zs: np.ndarray,
    F: np.ndarray,
    H: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    x0: np.ndarray,
) -> np.ndarray:
    """Run a basic Kalman filter and return the state history."""
    kf = KalmanFilter(dim_x=len(x0), dim_z=zs.shape[1])
    kf.x = x0
    kf.F = F
    kf.H = H
    kf.Q = Q
    kf.R = R
    xs = []
    for z in zs:
        kf.predict()
        kf.update(z)
        xs.append(kf.x.copy())
    return np.array(xs)


def kalman_with_residuals(
    zs: np.ndarray,
    F: np.ndarray,
    H: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    x0: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Run Kalman filter and also collect the residuals."""
    kf = KalmanFilter(dim_x=len(x0), dim_z=zs.shape[1])
    kf.x = x0
    kf.F = F
    kf.H = H
    kf.Q = Q
    kf.R = R
    xs = []
    residuals = []
    for z in zs:
        kf.predict()
        residual = z - (kf.H @ kf.x)
        kf.update(z)
        xs.append(kf.x.copy())
        residuals.append(residual)
    return np.array(xs), np.array(residuals)


# ---------------------------------------------------------------------------
# Plotting utilities
# ---------------------------------------------------------------------------

def _setup_plot(title: str, xlabel: str, ylabel: str) -> None:
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)


def plot_ned_positions(
    time: Iterable,
    ned: Iterable,
    label: str,
    title: str,
    outfile: str,
) -> None:
    plt.figure()
    for i, comp in enumerate("NED"):
        plt.plot(time, [r[i] for r in ned], label=f"{label} {comp}")
    _setup_plot(title, "Time (s)", "Position (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_residuals(
    time: Iterable,
    residuals: Sequence[Sequence[float]],
    title: str,
    outfile: str,
) -> None:
    res = list(residuals)
    plt.figure(figsize=(8, 6))
    components = ["North", "East", "Down"]
    for i in range(3):
        plt.plot(time, [r[i] for r in res], label=f"pos {components[i]}")
    for i in range(3):
        plt.plot(time, [r[i + 3] for r in res], linestyle="--", label=f"vel {components[i]}")
    _setup_plot(title, "Time (s)", "Residual")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_attitude(
    time: Iterable,
    yaw: Sequence[float],
    pitch: Sequence[float],
    roll: Sequence[float],
    title: str,
    outfile: str,
) -> None:
    plt.figure()
    plt.plot(time, roll, label="Roll")
    plt.plot(time, pitch, label="Pitch")
    plt.plot(time, yaw, label="Yaw")
    _setup_plot(title, "Time (s)", "Angle (deg)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


# ---------------------------------------------------------------------------
# Main demo routine
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="IMU/GNSS fusion demo")
    parser.add_argument("--gnss-file", default="GNSS_X001.csv", help="GNSS CSV file")
    parser.add_argument("--imu-file", default="IMU_X001.dat", help="IMU dat file")
    args = parser.parse_args()

    gnss = load_gnss_csv(args.gnss_file)
    _ = load_imu_dat(args.imu_file)  # IMU data not used in this simple demo

    lat = float(gnss.iloc[0].get("Latitude_deg", 0.0))
    lon = float(gnss.iloc[0].get("Longitude_deg", 0.0))
    C = compute_C_ECEF_to_NED(np.deg2rad(lat), np.deg2rad(lon))
    q = rot_to_quaternion(C.T)
    print("Initial quaternion (body to NED):", q)

    if {
        "X_ECEF_m",
        "Y_ECEF_m",
        "Z_ECEF_m",
        "VX_ECEF_mps",
        "VY_ECEF_mps",
        "VZ_ECEF_mps",
    } <= set(gnss.columns):
        z = gnss[
            [
                "X_ECEF_m",
                "Y_ECEF_m",
                "Z_ECEF_m",
                "VX_ECEF_mps",
                "VY_ECEF_mps",
                "VZ_ECEF_mps",
            ]
        ].values
    else:
        z = np.zeros((len(gnss), 6))

    times = gnss["Posix_Time"].values
    dt = float(np.mean(np.diff(times))) if len(times) > 1 else 1.0

    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    H = np.eye(6)
    Q = np.eye(6) * 0.01
    R = np.eye(6) * 0.1
    xs, residuals = kalman_with_residuals(z, F, H, Q, R, z[0])

    time_rel = times - times[0]
    plot_ned_positions(
        time_rel,
        xs[:, :3],
        "KF",
        "Kalman Filter Estimated Position",
        "positions.pdf",
    )
    plot_residuals(time_rel, residuals, "Kalman Filter Residuals", "residuals.pdf")

    yaw = np.degrees(np.arctan2(z[:, 4], z[:, 3]))
    pitch = np.zeros_like(yaw)
    roll = np.zeros_like(yaw)
    plot_attitude(time_rel, yaw, pitch, roll, "Attitude Angles", "attitude.pdf")

    print("Plots written: positions.pdf, residuals.pdf, attitude.pdf")


if __name__ == "__main__":
    main()
