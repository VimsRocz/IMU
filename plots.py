import numpy as np
try:
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None

def plot_frame(frame, t_truth, pos_truth, vel_truth, acc_truth, t_est, pos_est, vel_est, acc_est, out_dir):
    """Save 3x3 comparison plots for a single frame."""
    if plt is None:
        return
    labels = {'NED': ['N','E','D'], 'ECEF': ['X','Y','Z'], 'BODY': ['X','Y','Z']}[frame]
    fig, axes = plt.subplots(3, 3, figsize=(12, 8))
    for i, (data_truth, data_est) in enumerate(zip([pos_truth, vel_truth, acc_truth], [pos_est, vel_est, acc_est])):
        for j in range(3):
            ax = axes[i, j]
            if len(t_truth):
                ax.plot(t_truth, data_truth[:, j], 'k-', label='Truth')
            ax.plot(t_est, data_est[:, j], 'b-', label='Estimate')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel(labels[j])
            if i == 0:
                ax.set_title(f'Position {labels[j]}')
            elif i == 1:
                ax.set_title(f'Velocity {labels[j]}')
            else:
                ax.set_title(f'Acceleration {labels[j]}')
            ax.grid(True)
            if j == 0:
                ax.legend()
    fig.tight_layout()
    fig.savefig(f"{out_dir}/Task5_compare_{frame}.png")
    plt.close(fig)
