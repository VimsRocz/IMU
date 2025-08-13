from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def save_task2_summary_png(
    imu_data: np.ndarray,
    static_start: int,
    static_end: int,
    g_body: np.ndarray,
    omega_ie_body: np.ndarray,
    run_id: str,
    out_dir: str | Path,
) -> Path:
    """Save a Task 2 summary figure.

    The figure contains the accelerometer and gyroscope norms over time with the
    detected static interval highlighted as well as bar charts of the measured
    gravity and Earth rotation vectors in the body frame.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    time = imu_data[:, 1]
    gyro = imu_data[:, 2:5]
    acc = imu_data[:, 5:8]
    t = time - time[0]
    acc_norm = np.linalg.norm(acc, axis=1)
    gyro_norm = np.linalg.norm(gyro, axis=1)

    fig, axes = plt.subplots(2, 2, figsize=(10, 6))

    ax = axes[0, 0]
    ax.plot(t, acc_norm)
    ax.axvspan(t[static_start], t[static_end], color="red", alpha=0.3)
    ax.set_ylabel("|acc| [m/s²]")
    ax.set_title("Accelerometer norm")

    ax = axes[1, 0]
    ax.plot(t, gyro_norm)
    ax.axvspan(t[static_start], t[static_end], color="red", alpha=0.3)
    ax.set_ylabel("|gyro| [rad/s]")
    ax.set_xlabel("Time [s]")
    ax.set_title("Gyroscope norm")

    axes[0, 1].bar(["x", "y", "z"], g_body)
    axes[0, 1].set_title("g_body [m/s²]")

    axes[1, 1].bar(["x", "y", "z"], omega_ie_body)
    axes[1, 1].set_title("omega_ie_body [rad/s]")

    fig.suptitle("Task 2 – Body-frame vector summary")
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    out_png = out_dir / f"{run_id}_task2_summary.png"
    fig.savefig(out_png, dpi=300)
    plt.close(fig)
    return out_png
