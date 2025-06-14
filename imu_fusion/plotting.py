import matplotlib.pyplot as plt
from typing import Iterable, Sequence


def _setup_plot(title: str, xlabel: str, ylabel: str) -> None:
    """Common plot setup helper."""
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)


def plot_ned_positions(time: Iterable, ned: Iterable, label: str, title: str, outfile: str) -> None:
    plt.figure()
    for i, comp in enumerate("NED"):
        plt.plot(time, [r[i] for r in ned], label=f"{label} {comp}")
    _setup_plot(title, "Time (s)", "Position (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_residuals(time: Iterable, residuals: Sequence[Sequence[float]], title: str, outfile: str) -> None:
    """Plot position and velocity residuals."""
    res = list(residuals)
    plt.figure(figsize=(8, 6))
    components = ["North", "East", "Down"]
    for i in range(3):
        plt.plot(time, [r[i] for r in res], label=f"pos {components[i]}")
    for i in range(3):
        plt.plot(time, [r[i+3] for r in res], linestyle="--", label=f"vel {components[i]}")
    _setup_plot(title, "Time (s)", "Residual")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_attitude(time: Iterable, yaw: Sequence[float], pitch: Sequence[float], roll: Sequence[float], title: str, outfile: str) -> None:
    """Plot attitude angles over time."""

    _setup_plot(title, "Time (s)", "Angle (deg)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()
