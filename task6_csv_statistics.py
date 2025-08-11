#!/usr/bin/env python3
"""Task 6 – Basic CSV statistics and visualization.

This script extends the simple Task 5 example by loading a CSV file,
computing mean, median and mode of the first numerical column and
saving a histogram under ``results/``. Debugging information is
printed at each step to help trace execution.

Usage:
    python task6_csv_statistics.py [--csv-file data_task6.csv]

"""

from __future__ import annotations

import argparse
from pathlib import Path
from statistics import mean, median, mode

import matplotlib.pyplot as plt
import pandas as pd
from python.utils.save_plot_all import save_plot_all


def task6(csv_file: str = "data_task6.csv", output_dir: Path | str = "results") -> tuple[float | None, float | None, float | None]:
    """Read *csv_file*, compute statistics and save a histogram.

    Parameters
    ----------
    csv_file : str, optional
        Path to the CSV file containing a single numerical column.
    output_dir : str or Path, optional
        Directory where the histogram figure will be saved.

    Returns
    -------
    mean_val : float or None
    median_val : float or None
    mode_val : float or None
        Statistical measures of the data or ``None`` if a failure occurred.
    """

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    print("Starting Task 6: Reading and visualizing data from", csv_file)

    try:
        print(f"Attempting to read '{csv_file}' ...")
        data = pd.read_csv(csv_file)
        print("File read successfully. Checking data contents ...")
    except FileNotFoundError:
        print(f"Error: The file '{csv_file}' was not found.")
        return None, None, None
    except Exception as exc:  # pragma: no cover - unexpected errors
        print(f"Error: An unexpected issue occurred while reading the file: {exc}")
        return None, None, None

    column_name = data.columns[0]
    if not pd.api.types.is_numeric_dtype(data[column_name]):
        print("Error: The file contains invalid non-numerical data.")
        return None, None, None
    print(f"Data validation passed. Working with column: {column_name}")

    numerical_data = data[column_name].tolist()
    print("Extracted numerical data (first 10 values):", numerical_data[:10])

    print("First few rows of data:\n", data.head(5).to_string(index=False))

    try:
        data_mean = mean(numerical_data)
        data_median = median(numerical_data)
        data_mode = mode(numerical_data)
        print("Statistical calculations completed successfully.")
    except Exception as exc:  # pragma: no cover - rare
        print(f"Error: Failed to calculate statistics: {exc}")
        return None, None, None

    print(f"Mean of the data: {data_mean}")
    print(f"Median of the data: {data_median}")
    print(f"Mode of the data: {data_mode}")

    print("Generating histogram ...")
    plt.figure(figsize=(8, 6))
    plt.hist(numerical_data, bins=10, color="skyblue", edgecolor="black")
    plt.title(f"Histogram of {column_name}")
    plt.xlabel("Value")
    plt.ylabel("Frequency")
    plt.grid(True, linestyle="--", alpha=0.7)
    base = output_path / "task6_histogram"
    save_plot_all(plt.gcf(), str(base), show_plot=True)
    print(f"Histogram saved as {base.with_suffix('.png')} and .pickle")

    print("Returning statistical measures: (mean, median, mode)")
    return data_mean, data_median, data_mode


def main() -> None:
    parser = argparse.ArgumentParser(description="Task 6 CSV statistics")
    parser.add_argument("--csv-file", default="data_task6.csv", help="input CSV file")
    parser.add_argument("--output-dir", default="results", help="directory for plots")
    args = parser.parse_args()

    result = task6(args.csv_file, args.output_dir)
    if result != (None, None, None):
        print("Task 6 completed successfully with results:", result)
    else:
        print("Task 6 failed. Check messages above for details.")


if __name__ == "__main__":
    main()
