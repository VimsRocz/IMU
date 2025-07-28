"""Task 7 -- Analyze and visualise temperature data.

Usage:
    python task7_temperature_analysis.py --csv-file temperatures.csv \
        --output-dir results

This script reads a CSV file containing a single column of temperature
measurements in degrees Celsius, filters the data to the range
``0``–``30``\u00b0C and computes basic statistics. A histogram with a
vertical line at the mean is saved under ``results/``. The function
contains print statements for debugging and mirrors the MATLAB stub
``task7_temperature_analysis.m``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from statistics import mean, median

import matplotlib.pyplot as plt
import pandas as pd


def task7(csv_file: str = "temperatures.csv", output_dir: Path | str = "results") -> tuple[float | None, float | None]:
    """Read *csv_file*, filter values, compute statistics and save a histogram.

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
        Statistical measures of the filtered data or ``None`` if a failure
        occurred.
    """

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    print(
        "Starting Task 7: Reading, filtering, and visualising temperature data from '"
        f"{csv_file}'..."
    )

    try:
        print(f"Attempting to read '{csv_file}' ...")
        data = pd.read_csv(csv_file)
        print("File read successfully. Checking data contents ...")
    except FileNotFoundError:
        print(f"Error: The file '{csv_file}' was not found.")
        return None, None
    except Exception as exc:  # pragma: no cover - unexpected errors
        print(f"Error: An unexpected issue occurred while reading the file: {exc}")
        return None, None

    column_name = data.columns[0]
    if not pd.api.types.is_numeric_dtype(data[column_name]):
        print("Error: The file contains invalid non-numerical data.")
        return None, None
    print(f"Data validation passed. Working with column: {column_name}")

    temperatures = data[column_name].tolist()
    print("Extracted temperature data (first 10 values):", temperatures[:10])

    print("First few rows of data:\n", data.head(5).to_string(index=False))

    print("Filtering temperatures to include only values between 0\u00b0C and 30\u00b0C ...")
    filtered_temperatures = [temp for temp in temperatures if 0 <= temp <= 30]
    num_filtered = len(temperatures) - len(filtered_temperatures)
    print(f"Filtered out {num_filtered} value(s) outside the range [0\u00b0C, 30\u00b0C].")
    print(f"Remaining temperatures: {len(filtered_temperatures)}")

    if not filtered_temperatures:
        print(
            "Error: No temperature data remains after filtering. All values were outside the range [0\u00b0C, 30\u00b0C]."
        )
        return None, None

    try:
        data_mean = mean(filtered_temperatures)
        data_median = median(filtered_temperatures)
        print("Statistical calculations completed successfully.")
    except Exception as exc:  # pragma: no cover - rare
        print(f"Error: Failed to calculate statistics: {exc}")
        return None, None

    print(f"Mean of the filtered temperatures: {data_mean:.2f}\u00b0C")
    print(f"Median of the filtered temperatures: {data_median:.2f}\u00b0C")

    print("Generating histogram with mean line ...")
    plt.figure(figsize=(8, 6))
    plt.hist(filtered_temperatures, bins=10, color="lightgreen", edgecolor="black", alpha=0.7)
    plt.axvline(data_mean, color="red", linestyle="dashed", linewidth=2, label=f"Mean: {data_mean:.2f}\u00b0C")
    plt.title("Histogram of Filtered Temperatures")
    plt.xlabel("Temperature (\u00b0C)")
    plt.ylabel("Frequency")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.7)
    base = output_path / "task7_temperature_histogram"
    plt.savefig(base.with_suffix(".pdf"))
    plt.savefig(base.with_suffix(".png"))
    plt.close()
    print(f"Histogram saved as {base.with_suffix('.pdf')} and .png")

    print("Returning statistical measures: (mean, median)")
    return data_mean, data_median


def main() -> None:
    parser = argparse.ArgumentParser(description="Task 7 temperature analysis")
    parser.add_argument("--csv-file", default="temperatures.csv", help="input CSV file")
    parser.add_argument("--output-dir", default="results", help="directory for plots")
    args = parser.parse_args()

    result = task7(args.csv_file, args.output_dir)
    if result != (None, None):
        print(
            f"Task 7 completed successfully with results: Mean = {result[0]:.2f}\u00b0C, Median = {result[1]:.2f}\u00b0C"
        )
    else:
        print("Task 7 failed. Check messages above for details.")


if __name__ == "__main__":
    main()
