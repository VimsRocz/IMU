import numpy as np
import matplotlib.pyplot as plt


def save_zupt_variance(accel: np.ndarray, zupt_mask: np.ndarray, dt: float, dataset_id: str, threshold: float, window_size: int = 100) -> None:
    """Plot ZUPT-detected intervals and accelerometer variance."""
    t = np.arange(accel.shape[0]) * dt
    accel_norm = np.linalg.norm(accel, axis=1)
    mean_conv = np.ones(window_size) / window_size
    var = np.convolve(accel_norm ** 2, mean_conv, mode="same") - np.convolve(accel_norm, mean_conv, mode="same") ** 2

    plt.figure(figsize=(12, 4))
    plt.plot(t, var, label="Accel Norm Variance", color="tab:blue")
    plt.axhline(threshold, color="gray", linestyle="--", label="ZUPT threshold")
    plt.fill_between(t, 0, np.max(var), where=zupt_mask, color="tab:orange", alpha=0.3, label="ZUPT Detected")
    plt.xlabel("Time [s]")
    plt.ylabel("Variance")
    plt.tight_layout()
    plt.title("ZUPT Detection and Accelerometer Variance")
    filename = f"results/IMU_{dataset_id}_ZUPT_variance.pdf"
    plt.savefig(filename)
    plt.close()


def save_euler_angles(t: np.ndarray, euler_angles: np.ndarray, dataset_id: str) -> None:
    """Plot roll, pitch and yaw over time."""
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.tight_layout()
    plt.title("Attitude Angles (Roll/Pitch/Yaw) vs. Time")
    filename = f"results/IMU_{dataset_id}_EulerAngles_time.pdf"
    plt.savefig(filename)
    plt.close()


def save_residual_plots(t: np.ndarray, pos_filter: np.ndarray, pos_gnss: np.ndarray, vel_filter: np.ndarray, vel_gnss: np.ndarray, dataset_id: str) -> None:
    """Plot position and velocity residuals for N/E/D."""
    residual_pos = pos_filter - pos_gnss
    residual_vel = vel_filter - vel_gnss
    labels = ["North", "East", "Down"]
    for i, label in enumerate(labels):
        plt.figure()
        plt.plot(t, residual_pos[:, i])
        plt.xlabel("Time [s]")
        plt.ylabel("Position Residual [m]")
        plt.tight_layout()
        plt.title(f"Position Residuals ({label}) vs. Time")
        fname = f"results/IMU_{dataset_id}_GNSS_{dataset_id}_pos_residuals_{label}.pdf"
        plt.savefig(fname)
        plt.close()

        plt.figure()
        plt.plot(t, residual_vel[:, i])
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity Residual [m/s]")
        plt.tight_layout()
        plt.title(f"Velocity Residuals ({label}) vs. Time")
        fname = f"results/IMU_{dataset_id}_GNSS_{dataset_id}_vel_residuals_{label}.pdf"
        plt.savefig(fname)
        plt.close()


def save_attitude_over_time(t: np.ndarray, euler_angles: np.ndarray, dataset_id: str) -> None:
    """Plot roll, pitch and yaw over the entire dataset."""
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.tight_layout()
    plt.title("Attitude Angles (Roll/Pitch/Yaw) Over Time")
    fname = f"results/IMU_{dataset_id}_GNSS_{dataset_id}_attitude_time.pdf"
    plt.savefig(fname)
    plt.close()
