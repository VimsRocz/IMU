import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def save_plot(fig, outpath, title):
    ax = fig.axes[0] if fig.axes else fig.add_subplot(111)
    ax.set_title(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(outpath)
    plt.close(fig)


def plot_attitude(time, quats, outpath):
    r = R.from_quat(quats)
    euler = r.as_euler('xyz', degrees=True)
    fig, axs = plt.subplots(3, 1, figsize=(6, 8))
    labels = ['Roll', 'Pitch', 'Yaw']
    for i in range(3):
        axs[i].plot(time, euler[:, i])
        axs[i].set_ylabel(f"{labels[i]} (°)")
    axs[-1].set_xlabel("Time (s)")
    fig.suptitle("Attitude Angles Over Time")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(outpath)
    plt.close(fig)

import numpy as np


def plot_zupt_detection(accel_data: np.ndarray, gyro_data: np.ndarray,
                        zupt_mask: np.ndarray, sampling_period: float):
    """Plot acceleration and gyro norms with highlighted ZUPT segments."""
    t = np.arange(len(accel_data)) * sampling_period
    accel_norm = np.linalg.norm(accel_data, axis=1)
    gyro_norm = np.linalg.norm(gyro_data, axis=1)

    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(t, accel_norm, label="|accel|")
    axs[1].plot(t, gyro_norm, label="|gyro|")
    axs[0].set_ylabel("Accel Norm [m/s²]")
    axs[1].set_ylabel("Gyro Norm [rad/s]")
    axs[1].set_xlabel("Time [s]")

    for ax in axs:
        ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1],
                        where=zupt_mask, color='green', alpha=0.1,
                        label='ZUPT active')
        ax.legend()
        ax.grid(True)

    fig.tight_layout()
    return fig


def plot_initial_attitude(triad_rotation_matrices: dict):
    """Print and plot initial Euler angles from TRIAD results."""
    results = {}
    for ds, R_b2n in triad_rotation_matrices.items():
        rot = R.from_matrix(R_b2n)
        euler_deg = rot.as_euler('zyx', degrees=True)
        results[ds] = euler_deg
        fig, ax = plt.subplots()
        ax.bar(['Yaw', 'Pitch', 'Roll'], euler_deg)
        ax.set_ylabel('Degrees')
        ax.set_title(f'Initial Attitude (TRIAD) for {ds}')
        fig.tight_layout()
        plt.close(fig)
        print(
            f"Dataset {ds}: Initial Yaw={euler_deg[0]:.2f}°, "
            f"Pitch={euler_deg[1]:.2f}°, Roll={euler_deg[2]:.2f}°"
        )
    return results
