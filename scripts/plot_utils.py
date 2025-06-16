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
