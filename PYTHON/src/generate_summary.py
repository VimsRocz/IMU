"""Assemble a PNG report summarising all fusion runs.

Usage:
    python generate_summary.py

The script reads ``results/summary.csv`` produced by ``summarise_runs.py`` and
creates ``results/project_summary.png`` with one panel per dataset. Each panel
lists the RMSE and final error metrics and embeds the corresponding plot PNG if
available. This utility mirrors the MATLAB helper ``validate_pipeline_summary.m``
for cross-language comparison.
"""

import os
import logging
import pandas as pd
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.INFO, format="%(message)s")

# Load run results produced by ``summarise_runs.py``
DF_PATH = "results/summary.csv"


def main():
    os.makedirs("results", exist_ok=True)
    logging.info("Ensured 'results/' directory exists.")
    df = pd.read_csv(DF_PATH)

    if df.empty:
        logging.warning("No rows found in summary CSV; nothing to summarise.")
        return

    nrows = len(df)
    fig, axes = plt.subplots(nrows=nrows, figsize=(8.27, 3 * nrows))
    axes = [axes] if nrows == 1 else axes

    for ax, (_, row) in zip(axes, df.iterrows()):
        ax.axis("off")
        ds = f"{row['imu']} / {row['gnss']}"
        ax.text(
            0.01,
            0.95,
            f"{ds} - {row['method']}",
            fontsize=16,
            weight="bold",
            va="top",
            transform=ax.transAxes,
        )
        ax.text(
            0.01,
            0.75,
            f"RMSEpos = {row['rmse_pos']} m, Final Error = {row['final_pos']} m",
            fontsize=12,
            va="top",
            transform=ax.transAxes,
        )
        base = os.path.splitext(row["imu"])[0]
        image = f"results/{base}_{row['method']}_plots.png"
        if os.path.exists(image):
            img = plt.imread(image)
            ax.imshow(
                img,
                extent=(0.01, 0.99, 0.0, 0.7),
                transform=ax.transAxes,
                aspect="auto",
            )

    fig.tight_layout()
    fig.savefig("results/project_summary.png", dpi=150)


if __name__ == '__main__':
    main()
