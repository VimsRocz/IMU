from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import uuid


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


def task2_measure_body_vectors(
    imu_data,
    static_indices: tuple[int, int],
    output_dir: str | Path,
) -> Path:
    """Plot measured gravity and Earth rotation vectors with error bars."""

    start, end = static_indices
    plot_id = uuid.uuid4().hex
    print(f"Task 2 plot ID: {plot_id}")
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        acc_cols = ["accel_x", "accel_y", "accel_z"]
        gyro_cols = ["gyro_x", "gyro_y", "gyro_z"]
        static_df = imu_data.iloc[start:end]
        acc = static_df[acc_cols].to_numpy()
        gyro = static_df[gyro_cols].to_numpy()
        g_body = -np.mean(acc, axis=0)
        omega_ie_body = np.mean(gyro, axis=0)
        acc_err = np.std(acc, axis=0)
        gyro_err = np.std(gyro, axis=0)
        errors = np.concatenate([acc_err, gyro_err])
    except Exception:
        g_body = np.zeros(3)
        omega_ie_body = np.zeros(3)
        errors = np.full(6, 1e-6)
        print("Error data not available; using defaults")
    else:
        if np.all(errors == 0):
            errors = np.full(6, 1e-6)
            print("Error data not available; using defaults")

    labels = [
        "Gravity X",
        "Gravity Y",
        "Gravity Z",
        "Earth Rot X",
        "Earth Rot Y",
        "Earth Rot Z",
    ]
    values = np.concatenate([g_body, omega_ie_body])

    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(len(labels))
    bars = ax.bar(x, values, yerr=errors, capsize=5)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Value")
    ax.set_title(
        f"Task 2: Measured Vectors in Body Frame with Errors\nID: {plot_id}"
    )

    for bar, val in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{val:.2e}",
            ha="center",
            va="bottom",
            fontsize=8,
        )

    fig.tight_layout()
    out_png = out_dir / "IMU_X002_GNSS_X002_TRIAD_task2_vectors.png"
    fig.savefig(out_png, dpi=300)
    plt.close(fig)
    print(f"Task 2: saved plot -> {out_png}")
    return out_png
