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
from typing import Sequence

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R


def _find_cols(df: pd.DataFrame, options: Sequence[Sequence[str]]) -> Sequence[str]:
    """Return the first column set that is fully present in *df*."""
    for cols in options:
        if all(c in df.columns for c in cols):
            return list(cols)
    raise KeyError(f"None of the column sets {options!r} found in {list(df.columns)}")


def load_data(est_path: str, gnss_path: str) -> tuple[pd.DataFrame, pd.DataFrame]:
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


def compute_residuals(est: pd.DataFrame, gnss: pd.DataFrame) -> tuple[pd.DataFrame, np.ndarray]:
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
    os.makedirs(out_dir, exist_ok=True)

    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for i, comp in enumerate(['X', 'Y', 'Z']):
        ax[i].plot(res_df.index, res_df['pos'][comp])
        ax[i].set_ylabel(f'{comp} [m]')
        ax[i].grid(True)
    ax[-1].set_xlabel('Time (s)')
    fig.suptitle('Position Residuals')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'p_residuals.png'), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for i, comp in enumerate(['X', 'Y', 'Z']):
        ax[i].plot(res_df.index, res_df['vel'][comp])
        ax[i].set_ylabel(f'{comp} [m/s]')
        ax[i].grid(True)
    ax[-1].set_xlabel('Time (s)')
    fig.suptitle('Velocity Residuals')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'v_residuals.png'), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for i, comp in enumerate(['roll', 'pitch', 'yaw']):
        ax[i].plot(res_df.index, res_df['att'][comp])
        ax[i].set_ylabel(f'{comp} [deg]')
        ax[i].grid(True)
    ax[-1].set_xlabel('Time (s)')
    fig.suptitle('Attitude Angles')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'attitude_angles.png'), dpi=150)
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
    res_df, _ = compute_residuals(est, gnss)
    print_stats(res_df)
    plot_residuals(res_df, args.output)


if __name__ == '__main__':
    main()
