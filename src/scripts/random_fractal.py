import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import rgb2hex
from typing import Optional


def generate_colors(num_points: int, colormap: str = "hsv"):
    """Return a list of hex colors sampled from the given colormap."""
    cmap = plt.get_cmap(colormap)
    return [rgb2hex(cmap(i / num_points)) for i in range(num_points)]


def chaos_game(num_points: int = 50000):
    """Generate points using the chaos game to create a Sierpinski triangle."""
    vertices = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.5, np.sqrt(3) / 2],
    ])
    p = np.random.rand(2)
    points = np.zeros((num_points, 2))
    for i in range(num_points):
        v = vertices[np.random.randint(0, 3)]
        p = (p + v) / 2
        points[i] = p
    return points


def plot_fractal(num_points: int = 50000, outfile: Optional[str] = None) -> None:
    """Plot a random fractal with a hex-based spectral gradient.

    If ``outfile`` is provided the figure is saved instead of shown. This allows
    the script to run in headless environments.
    """

    pts = chaos_game(num_points)
    colors = generate_colors(num_points)
    plt.figure(figsize=(6, 6))
    plt.scatter(pts[:, 0], pts[:, 1], s=0.1, c=colors, marker=".")
    plt.axis("equal")
    plt.axis("off")
    plt.tight_layout()

    if outfile:
        plt.savefig(outfile, dpi=300)
    else:
        plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Visualize a random fractal.")
    parser.add_argument(
        "-n",
        "--num-points",
        type=int,
        default=50000,
        help="Number of points to generate",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Path to save the figure instead of displaying it",
    )
    args = parser.parse_args()

    plot_fractal(num_points=args.num_points, outfile=args.output)
