"""Utility routines for automated figure generation and run summaries.

This module extends the batch processing script with helpers to create the
standard set of plots and a CSV/LaTeX table summarising RMSE and innovation
statistics.  Functions here are intentionally generic so that they can be
hooked into :mod:`run_all_datasets` or another driver script.
"""

from __future__ import annotations

import os
from typing import Iterable, Tuple

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from utils import compute_C_ECEF_to_NED

# ---------------------------------------------------------------------------
# Where figures and tables should be written
# ---------------------------------------------------------------------------
OUTPUT_DIR = "results/auto_plots"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Collect metrics for the summary table
summary_records: list[dict] = []


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def save_task3_plots(method_name: str, dataset_name: str, quat_df: pd.DataFrame,
                     angle_errors: pd.DataFrame) -> None:
    """Save Task‑III comparison figures.

    Parameters
    ----------
    method_name:
        Name of the attitude-initialisation method.
    dataset_name:
        Identifier of the processed dataset.
    quat_df:
        DataFrame with columns ``['component', 'case', 'value']`` representing
        quaternion components for different cases.
    angle_errors:
        DataFrame with columns ``['error_type', 'deg_error']``.
    """
    fig, ax = plt.subplots()
    quat_df.pivot(index="component", columns="case", values="value").plot.bar(ax=ax)
    ax.set_title(f"{method_name} quaternion components ({dataset_name})")
    fig.savefig(f"{OUTPUT_DIR}/{dataset_name}_{method_name}_task3_quat.png",
                dpi=200)
    plt.close(fig)

    fig, ax = plt.subplots()
    angle_errors.pivot(index="error_type", values="deg_error").plot.bar(ax=ax)
    ax.set_ylabel("Angle error [deg]")
    ax.set_title(f"{method_name} attitude error ({dataset_name})")
    fig.savefig(f"{OUTPUT_DIR}/{dataset_name}_{method_name}_task3_error.png",
                dpi=200)
    plt.close(fig)


def save_task4_plots(method_name: str, dataset_name: str, gnss_ned: pd.DataFrame,
                     imu_ned: pd.DataFrame) -> None:
    """Plot GNSS vs IMU-only position in N/E/D."""
    for comp in ["N", "E", "D"]:
        fig, ax = plt.subplots()
        ax.plot(gnss_ned.index, gnss_ned[comp], "-", label="GNSS")
        ax.plot(imu_ned.index, imu_ned[comp], "--", label="IMU only")
        ax.set_title(f"{method_name} {comp}-pos ({dataset_name})")
        ax.legend()
        fig.savefig(
            f"{OUTPUT_DIR}/{dataset_name}_{method_name}_task4_{comp}_pos.png",
            dpi=200,
        )
        plt.close(fig)


def save_task5_plots(method_name: str, dataset_name: str, fused: pd.DataFrame,
                     gnss: pd.DataFrame) -> None:
    """Plot Kalman filter fused output against GNSS."""
    for comp in ["N", "E", "D"]:
        fig, ax = plt.subplots()
        ax.plot(gnss.index, gnss[comp], "-", label="GNSS")
        ax.plot(fused.index, fused[comp], "--", label="Fused")
        ax.set_title(f"{method_name} {comp}-pos KF ({dataset_name})")
        ax.legend()
        fig.savefig(
            f"{OUTPUT_DIR}/{dataset_name}_{method_name}_task5_{comp}_pos.png",
            dpi=200,
        )
        plt.close(fig)


def save_validation_plots(
    method_name: str,
    dataset_name: str,
    residuals: pd.DataFrame,
    innovations: pd.DataFrame,
) -> None:
    """Store residual and innovation time series."""
    for name, df in ("residuals", residuals), ("innovations", innovations):
        fig, ax = plt.subplots()
        df.plot(ax=ax)
        ax.set_title(f"{method_name} {name} ({dataset_name})")
        fig.savefig(f"{OUTPUT_DIR}/{dataset_name}_{method_name}_{name}.png",
                    dpi=200)
        plt.close(fig)


# ---------------------------------------------------------------------------
# Summary table helpers
# ---------------------------------------------------------------------------

def record_summary(dataset_name: str, method_name: str, rmse: float,
                   final_error: float, innov_std: float) -> None:
    """Record metrics for later export."""
    summary_records.append(
        {
            "dataset": dataset_name,
            "method": method_name,
            "rmse_pos": rmse,
            "final_pos_err": final_error,
            "innov_std": innov_std,
        }
    )


def export_summary_table() -> None:
    """Write the collected metrics to ``summary_metrics.csv`` and ``.tex``."""
    df = pd.DataFrame(summary_records)
    df.to_csv(f"{OUTPUT_DIR}/summary_metrics.csv", index=False)
    with open(f"{OUTPUT_DIR}/summary_metrics.tex", "w") as fh:
        fh.write(df.to_latex(index=False, float_format="%.2f"))


# ---------------------------------------------------------------------------
# Placeholder processing functions
# ---------------------------------------------------------------------------

def load_data(dataset: Tuple[str, str]):
    """Load IMU and GNSS logs.

    Parameters
    ----------
    dataset : tuple(str, str)
        ``(imu_file, gnss_file)`` pair describing the dataset.

    Returns
    -------
    tuple of :class:`pandas.DataFrame`
        ``(gnss_df, imu_df)`` with the raw measurements.

    Notes
    -----
    This is a lightweight loader used by :func:`run_batch`.  It simply reads
    the GNSS CSV with :func:`pandas.read_csv` and the IMU ``.dat`` file with
    :func:`numpy.loadtxt` and wraps the result into a DataFrame.  Real projects
    will likely need a more elaborate parser but this keeps the example
    self‑contained.
    """
    imu_file, gnss_file = dataset
    gnss = pd.read_csv(gnss_file)
    imu = pd.DataFrame(np.loadtxt(imu_file))
    return gnss, imu


def task3_attitude_comparison(*args, **kwargs):  # pragma: no cover - placeholder
    """Dummy attitude comparison.

    The real project computes quaternion initialisation accuracy for each
    method.  Here we simply return an identity quaternion and zero errors so
    that the plotting helpers have something to work with.
    """
    method = kwargs.get("method", "unknown")
    quat_df = pd.DataFrame(
        {
            "component": ["w", "x", "y", "z"],
            "case": method,
            "value": [1.0, 0.0, 0.0, 0.0],
        }
    )
    angle_err = pd.DataFrame(
        {
            "error_type": ["roll", "pitch", "yaw"],
            "deg_error": [0.0, 0.0, 0.0],
        }
    )
    return quat_df, angle_err


def task4_gnss_vs_imu(gnss: pd.DataFrame, imu: pd.DataFrame, *_, **__):
    """Return GNSS position in NED and a placeholder IMU solution."""
    if {"X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"}.issubset(gnss.columns):
        r_ecef = gnss[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].to_numpy()
        lat0 = np.deg2rad(gnss["Latitude_deg"].iloc[0])
        lon0 = np.deg2rad(gnss["Longitude_deg"].iloc[0])
        r0 = r_ecef[0]
        C = compute_C_ECEF_to_NED(lat0, lon0)
        ned = (r_ecef - r0) @ C.T
        gnss_ned = pd.DataFrame(ned, columns=["N", "E", "D"], index=gnss["Posix_Time"])
    else:
        # fall back to first three columns as position
        pos = gnss.iloc[:, :3].to_numpy()
        gnss_ned = pd.DataFrame(pos - pos[0], columns=["N", "E", "D"], index=gnss.index)
    imu_ned = gnss_ned.copy()  # placeholder integration result
    return gnss_ned, imu_ned


def task5_filter(gnss: pd.DataFrame, *_args, **__):
    """Return a dummy Kalman filter output matching the GNSS track."""
    fused = gnss.copy()
    return fused, gnss


def compute_validation(fused: pd.DataFrame, gnss: pd.DataFrame):
    """Compute simple residuals/innovations for demonstration."""
    common = [c for c in ["N", "E", "D", "x", "y", "z"] if c in fused.columns and c in gnss.columns]
    res = fused[common] - gnss[common]
    innovations = res.diff().fillna(0.0)
    return res, innovations


def compute_rmse(fused: pd.DataFrame, gnss: pd.DataFrame) -> float:
    """Return the position RMSE between *fused* and *gnss* tracks."""
    cols = [c for c in ["N", "E", "D", "x", "y", "z"] if c in fused.columns and c in gnss.columns]
    diff = fused[cols].to_numpy() - gnss[cols].to_numpy()
    return float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))


# ---------------------------------------------------------------------------
# Example batch driver
# ---------------------------------------------------------------------------

def run_batch(all_datasets: Iterable[Tuple[str, str]],
              methods: Iterable[str]) -> None:
    """Skeleton batch loop using the helpers in this module."""
    for ds in all_datasets:
        gnss, imu = load_data(ds)
        for method in methods:
            quat_df, angle_err = task3_attitude_comparison(gnss, imu, method)
            save_task3_plots(method, ds[0], quat_df, angle_err)

            gnss_ned, imu_ned = task4_gnss_vs_imu(gnss, imu, method)
            save_task4_plots(method, ds[0], gnss_ned, imu_ned)

            fused, gnss_interp = task5_filter(gnss, imu, method)
            save_task5_plots(method, ds[0], fused, gnss_interp)

            residuals, innovations = compute_validation(fused, gnss_interp)
            save_validation_plots(method, ds[0], residuals, innovations)

            rmse = compute_rmse(fused, gnss_interp)
            final_err = (fused.iloc[-1] - gnss_interp.iloc[-1]).pow(2).sum() ** 0.5
            innov_std = innovations.std().mean()
            record_summary(ds[0], method, rmse, final_err, innov_std)

    export_summary_table()
