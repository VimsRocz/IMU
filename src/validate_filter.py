#!/usr/bin/env python3
"""Validate filter output against GNSS updates.

This script merges the saved filter state history with GNSS measurements,
computes position/velocity residuals and plots them.  It also converts the
stored quaternions to Euler angles for a quick visual check of the attitude
solution.
"""

from __future__ import annotations

import argparse
import os
import logging
from typing import Sequence, Tuple

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

logging.basicConfig(level=logging.INFO, format="%(message)s")
os.makedirs('results', exist_ok=True)
logging.info("Ensured 'results/' directory exists.")


def _find_cols(df: pd.DataFrame, options: Sequence[Sequence[str]]) -> Sequence[str]:
    """Return the first column set that is fully present in *df*."""
    for cols in options:
        if all(c in df.columns for c in cols):
            return list(cols)
    raise KeyError(f"None of the column sets {options!r} found in {list(df.columns)}")


def load_data(est_path: str, gnss_path: str) -> Tuple[pd.DataFrame, pd.DataFrame]:
    est = pd.read_csv(est_path)
    gnss = pd.read_csv(gnss_path)

    # unify time column names
    for df in (est, gnss):
        if 'time' in df.columns:
            continue
        for cand in ['t', 'timestamp', 'Posix_Time']:
            if cand in df.columns:
                df.rename(columns={cand: 'time'}, inplace=True)
                break
        else:
            raise KeyError('No time column found in input CSV')

    return est, gnss


def compute_residuals(est: pd.DataFrame, gnss: pd.DataFrame) -> Tuple[pd.DataFrame, np.ndarray]:
    """Merge filter estimates with GNSS and return residual arrays."""
    pos_cols_est = _find_cols(est, [
        ['x', 'y', 'z'],
        ['px', 'py', 'pz'],
        ['pos_x', 'pos_y', 'pos_z'],
    ])
    vel_cols_est = _find_cols(est, [
        ['vx', 'vy', 'vz'],
        ['vel_x', 'vel_y', 'vel_z'],
    ])
    quat_cols = _find_cols(est, [
        ['qw', 'qx', 'qy', 'qz'],
        ['q0', 'q1', 'q2', 'q3'],
    ])

    pos_cols_gnss = _find_cols(gnss, [
        ['x', 'y', 'z'],
        ['X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'],
        ['pos_x', 'pos_y', 'pos_z'],
    ])
    vel_cols_gnss = _find_cols(gnss, [
        ['vx', 'vy', 'vz'],
        ['VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps'],
        ['vel_x', 'vel_y', 'vel_z'],
    ])

    est_sorted = est.sort_values('time')
    gnss_sorted = gnss.sort_values('time')
    merged = pd.merge_asof(gnss_sorted, est_sorted, on='time', direction='nearest')

    r_pos = merged[pos_cols_est].to_numpy() - merged[pos_cols_gnss].to_numpy()
    r_vel = merged[vel_cols_est].to_numpy() - merged[vel_cols_gnss].to_numpy()

    quat = merged[quat_cols].to_numpy()
    rot = R.from_quat(quat[:, [1, 2, 3, 0]])  # w,x,y,z -> x,y,z,w for scipy
    euler = rot.as_euler('xyz', degrees=True)

    res_df = pd.DataFrame(
        r_pos, columns=['X', 'Y', 'Z'], index=merged['time']
    )
    vel_df = pd.DataFrame(
        r_vel, columns=['X', 'Y', 'Z'], index=merged['time']
    )
    att_df = pd.DataFrame(
        euler, columns=['roll', 'pitch', 'yaw'], index=merged['time']
    )

    return pd.concat({'pos': res_df, 'vel': vel_df, 'att': att_df}, axis=1), merged['time']


def plot_residuals(res_df: pd.DataFrame, out_dir: str) -> None:
    """Plot position, velocity and acceleration residuals in a 3×3 grid."""
    os.makedirs(out_dir, exist_ok=True)

    t = res_df.index.to_numpy()
    vel = res_df['vel'][['X', 'Y', 'Z']].to_numpy()
    acc = np.gradient(vel, t, axis=0)

    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)
    comps = ['X', 'Y', 'Z']
    datasets = [
        (res_df['pos'][comps].to_numpy(), 'Position Residual [m]'),
        (vel, 'Velocity Residual [m/s]'),
        (acc, 'Acceleration Residual [m/s$^2$]'),
    ]

    for row, (arr, ylab) in enumerate(datasets):
        for col, comp in enumerate(comps):
            ax = axes[row, col]
            ax.plot(t, arr[:, col])
            if row == 0:
                ax.set_title(comp)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel('Time (s)')
            ax.grid(True)

    fig.suptitle('Filter Residuals')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(out_dir, 'residuals.pdf'))
    plt.close(fig)


def plot_residuals_new(gnss_times: np.ndarray,
                       positions_pred: np.ndarray,
                       positions_meas: np.ndarray,
                       velocities_pred: np.ndarray,
                       velocities_meas: np.ndarray,
                       dataset: str,
                       method: str) -> None:
    """Plot residuals in a 3×3 grid and save as PDF."""
    res_pos = positions_meas - positions_pred
    res_vel = velocities_meas - velocities_pred
    acc = np.gradient(res_vel, gnss_times, axis=0)

    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)
    comps = ['X', 'Y', 'Z']
    datasets = [
        (res_pos, 'Position Residual [m]'),
        (res_vel, 'Velocity Residual [m/s]'),
        (acc, 'Acceleration Residual [m/s$^2$]'),
    ]

    for row, (arr, ylab) in enumerate(datasets):
        for col, comp in enumerate(comps):
            ax = axes[row, col]
            ax.plot(gnss_times, arr[:, col])
            if row == 0:
                ax.set_title(comp)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel('Time (s)')
            ax.grid(True)

    fig.suptitle(f'Residuals: {dataset} - {method}')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out = f'results/{dataset}_{method}_residuals.pdf'
    fig.savefig(out)
    plt.close(fig)


def plot_attitude(time: np.ndarray, quaternions: np.ndarray, dataset: str,
                  method: str) -> None:
    """Plot roll/pitch/yaw attitude angles over time."""
    from filterpy.common import q_to_euler
    rpy = np.array([q_to_euler(q) for q in quaternions])

    fig, axs = plt.subplots(3, 1, figsize=(8, 8))
    for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
        axs[i].plot(time, rpy[:, i])
        axs[i].set_title(f'{label} over Time')
    fig.suptitle(f'Attitude Angles: {dataset} - {method}')
    fig.tight_layout()
    fig.savefig(f'results/{dataset}_{method}_attitude.pdf')
    plt.close(fig)


def print_stats(res_df: pd.DataFrame) -> None:
    pos_stats = res_df['pos'].agg(['mean', 'std'])
    vel_stats = res_df['vel'].agg(['mean', 'std'])
    print('# Residual statistics')
    print('Position mean [m]:\n', pos_stats.loc['mean'].to_string())
    print('Position std [m]:\n', pos_stats.loc['std'].to_string())
    print('Velocity mean [m/s]:\n', vel_stats.loc['mean'].to_string())
    print('Velocity std [m/s]:\n', vel_stats.loc['std'].to_string())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--estimates', required=True, help='filter state CSV')
    ap.add_argument('--gnss', required=True, help='GNSS update CSV')
    ap.add_argument('--output', required=True, help='output directory for plots')
    args = ap.parse_args()

    est, gnss = load_data(args.estimates, args.gnss)
    res_df, times = compute_residuals(est, gnss)
    print_stats(res_df)
    plot_residuals(res_df, args.output)

    # Derive dataset/method names from file paths
    dataset = os.path.splitext(os.path.basename(args.gnss))[0]
    method = os.path.splitext(os.path.basename(args.estimates))[0]

    # Recompute predictions and measurements for new helper plots
    pos_cols_est = _find_cols(est, [
        ['x', 'y', 'z'],
        ['px', 'py', 'pz'],
        ['pos_x', 'pos_y', 'pos_z'],
    ])
    vel_cols_est = _find_cols(est, [
        ['vx', 'vy', 'vz'],
        ['vel_x', 'vel_y', 'vel_z'],
    ])
    quat_cols = _find_cols(est, [
        ['qw', 'qx', 'qy', 'qz'],
        ['q0', 'q1', 'q2', 'q3'],
    ])
    pos_cols_gnss = _find_cols(gnss, [
        ['x', 'y', 'z'],
        ['X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'],
        ['pos_x', 'pos_y', 'pos_z'],
    ])
    vel_cols_gnss = _find_cols(gnss, [
        ['vx', 'vy', 'vz'],
        ['VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps'],
        ['vel_x', 'vel_y', 'vel_z'],
    ])

    est_sorted = est.sort_values('time')
    gnss_sorted = gnss.sort_values('time')
    merged = pd.merge_asof(gnss_sorted, est_sorted, on='time', direction='nearest')

    positions_pred = merged[pos_cols_est].to_numpy()
    positions_meas = merged[pos_cols_gnss].to_numpy()
    velocities_pred = merged[vel_cols_est].to_numpy()
    velocities_meas = merged[vel_cols_gnss].to_numpy()
    quats = merged[quat_cols].to_numpy()

    plot_residuals_new(
        merged['time'].to_numpy(),
        positions_pred,
        positions_meas,
        velocities_pred,
        velocities_meas,
        dataset,
        method,
    )
    plot_attitude(merged['time'].to_numpy(), quats, dataset, method)


if __name__ == '__main__':
    main()
