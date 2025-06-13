import matplotlib.pyplot as plt
from typing import Iterable


def plot_ned_positions(time: Iterable, ned: Iterable, label: str, outfile: str):
    plt.figure()
    for i, comp in enumerate('NED'):
        plt.plot(time, [r[i] for r in ned], label=f'{label} {comp}')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(outfile)
    plt.close()
