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
    plt.figure()
    plt.plot(time, roll, label="Roll")
    plt.plot(time, pitch, label="Pitch")
    plt.plot(time, yaw, label="Yaw")
    _setup_plot(title, "Time (s)", "Angle (deg)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_ned_positions_multi(
    times: Sequence[Iterable],
    neds: Sequence[Iterable],
    labels: Sequence[str],
    title: str,
    outfile: str,
) -> None:
    """Plot NED positions for multiple datasets on one figure."""
    plt.figure()
    for t, ned, label in zip(times, neds, labels):
        for i, comp in enumerate("NED"):
            plt.plot(t, [r[i] for r in ned], label=f"{label} {comp}")
    _setup_plot(title, "Time (s)", "Position (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_residuals_multi(
    times: Sequence[Iterable],
    residual_sets: Sequence[Sequence[Sequence[float]]],
    labels: Sequence[str],
    title: str,
    outfile: str,
) -> None:
    """Plot residuals for multiple datasets on one figure."""
    plt.figure(figsize=(8, 6))
    components = ["North", "East", "Down"]
    for t, residuals, label in zip(times, residual_sets, labels):
        res = list(residuals)
        for i in range(3):
            plt.plot(t, [r[i] for r in res], label=f"{label} pos {components[i]}")
        for i in range(3):
            plt.plot(
                t,
                [r[i + 3] for r in res],
                linestyle="--",
                label=f"{label} vel {components[i]}",
            )
    _setup_plot(title, "Time (s)", "Residual")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()


def plot_attitude_multi(
    times: Sequence[Iterable],
    yaws: Sequence[Iterable],
    pitches: Sequence[Iterable],
    rolls: Sequence[Iterable],
    labels: Sequence[str],
    title: str,
    outfile: str,
) -> None:
    """Plot attitude angles for multiple datasets."""
    plt.figure()
    for t, yaw, pitch, roll, label in zip(times, yaws, pitches, rolls, labels):
        plt.plot(t, roll, label=f"{label} Roll")
        plt.plot(t, pitch, label=f"{label} Pitch")
        plt.plot(t, yaw, label=f"{label} Yaw")
    _setup_plot(title, "Time (s)", "Angle (deg)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()
