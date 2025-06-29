import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def plot_frame(frame: str,
               t_ref: np.ndarray,
               pos_ref: np.ndarray,
               vel_ref: np.ndarray,
               acc_ref: np.ndarray,
               t_est: np.ndarray,
               pos_est: np.ndarray,
               vel_est: np.ndarray,
               acc_est: np.ndarray,
               out_dir: str) -> None:
    """Plot reference vs estimated data in the given *frame*.

    Parameters
    ----------
    frame : str
        Name of the coordinate frame (e.g. ``'NED'``).
    t_ref, pos_ref, vel_ref, acc_ref : ndarray
        Time vector and reference position/velocity/acceleration.
    t_est, pos_est, vel_est, acc_est : ndarray
        Time vector and estimated position/velocity/acceleration.
    out_dir : str
        Directory where the PNG file is saved.
    """
    labels = {
        'NED': ['N', 'E', 'D'],
        'ECEF': ['X', 'Y', 'Z'],
        'Body': ['X', 'Y', 'Z'],
    }
    comp = labels.get(frame, ['X', 'Y', 'Z'])

    fig, axes = plt.subplots(3, 3, figsize=(12, 8), sharex='col')

    data_pairs = [
        (pos_ref, pos_est, 'Position'),
        (vel_ref, vel_est, 'Velocity'),
        (acc_ref, acc_est, 'Acceleration'),
    ]

    for row, (ref, est, title) in enumerate(data_pairs):
        for col in range(3):
            ax = axes[row, col]
            ax.plot(t_ref, ref[:, col], 'k-', label='truth')
            ax.plot(t_est, est[:, col], 'r--', label='fused')
            if row == 0:
                ax.set_title(comp[col])
            if col == 0:
                ax.set_ylabel(title)
            if row == 2:
                ax.set_xlabel('Time [s]')
            if row == 0 and col == 0:
                ax.legend()

    fig.suptitle(f'Task5 comparison in {frame} frame')
    fig.tight_layout()
    out_path = Path(out_dir) / f'Task5_compare_{frame.upper()}.png'
    fig.savefig(out_path)
    plt.close(fig)
