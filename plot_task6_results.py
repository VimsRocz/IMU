import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import os

# Ensure results directory exists
os.makedirs("results", exist_ok=True)

# Load data
mat_file = "results/IMU_X001_GNSS_X001_Davenport.mat"
if not os.path.exists(mat_file):
    raise FileNotFoundError(mat_file)

data = sio.loadmat(mat_file)

# Check available keys
keys = list(data.keys())
print(f"Available keys: {keys}")

# Use consistent keys
try:
    pos_ned = data["pos_ned_m"]
    vel_ned = data["vel_ned_ms"]
    att_quat = data["att_quat"]
    time_s = data["time_s"].squeeze()
    pos_ecef = data["pos_ecef_m"]
    vel_ecef = data["vel_ecef_ms"]
    pos_body = data["pos_body_m"]
    vel_body = data["vel_body_ms"]
except KeyError as e:
    print(f"KeyError: {e}. Check data keys.")
    raise


def plot_task6_results(
    pos_ned,
    vel_ned,
    att_quat,
    time_s,
    pos_ecef,
    vel_ecef,
    pos_body,
    vel_body,
    method="Davenport",
):
    # NED frame plots
    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["North", "East", "Down"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, pos_ned[:, i], label=label)
        plt.title(f"{method} Position {label} (m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/IMU_X001_GNSS_X001_{method}_task6_position_ned.pdf")
    plt.close()

    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["North", "East", "Down"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, vel_ned[:, i], label=label)
        plt.title(f"{method} Velocity {label} (m/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/IMU_X001_GNSS_X001_{method}_task6_velocity_ned.pdf")
    plt.close()

    # ECEF frame plots
    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["X", "Y", "Z"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, pos_ecef[:, i], label=label)
        plt.title(f"{method} Position {label} (ECEF, m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/IMU_X001_GNSS_X001_{method}_task6_position_ecef.pdf")
    plt.close()

    # Body frame plots
    plt.figure(figsize=(10, 8))
    for i, label in enumerate(["X", "Y", "Z"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_s, pos_body[:, i], label=label)
        plt.title(f"{method} Position {label} (Body, m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/IMU_X001_GNSS_X001_{method}_task6_position_body.pdf")
    plt.close()

    # Log results
    print(
        f"Subtask 6.8.2: Plotted {method} position North: First = {pos_ned[0, 0]:.4f}, Last = {pos_ned[-1, 0]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} position East: First = {pos_ned[0, 1]:.4f}, Last = {pos_ned[-1, 1]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} position Down: First = {pos_ned[0, 2]:.4f}, Last = {pos_ned[-1, 2]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} velocity North: First = {vel_ned[0, 0]:.4f}, Last = {vel_ned[-1, 0]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} velocity East: First = {vel_ned[0, 1]:.4f}, Last = {vel_ned[-1, 1]:.4f}"
    )
    print(
        f"Subtask 6.8.2: Plotted {method} velocity Down: First = {vel_ned[0, 2]:.4f}, Last = {vel_ned[-1, 2]:.4f}"
    )
    print(
        f"Subtask 6.8.2: {method} plot saved as 'results/IMU_X001_GNSS_X001_{method}_task6_results.pdf'"
    )


plot_task6_results(
    pos_ned, vel_ned, att_quat, time_s, pos_ecef, vel_ecef, pos_body, vel_body
)
print("Fused mixed frames plot saved")
print("All data in NED frame plot saved")
print("All data in ECEF frame plot saved")
print("All data in body frame plot saved")
