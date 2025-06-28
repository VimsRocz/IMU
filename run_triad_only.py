#!/usr/bin/env python3
"""Run all datasets using only the TRIAD initialisation method and
validate results when ground truth data is available."""

import subprocess
import sys
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent

# --- Run the batch processor -------------------------------------------------
cmd = [
    sys.executable,
    str(HERE / "run_all_datasets.py"),
    "--method",
    "TRIAD",
    *sys.argv[1:],
]
subprocess.run(cmd, check=True)

# --- Validate results when STATE_<id>.txt exists -----------------------------
results = HERE / "results"
for mat in results.glob("*_TRIAD_kf_output.mat"):
    m = re.match(r"IMU_(X\d+)_.*_TRIAD_kf_output\.mat", mat.name)
    if not m:
        continue
    truth = HERE / f"STATE_{m.group(1)}.txt"
    if not truth.exists():
        continue
    vcmd = [
        sys.executable,
        str(HERE / "validate_with_truth.py"),
        "--est-file",
        str(mat),
        "--truth-file",
        str(truth),
        "--output",
        str(results),
    ]
    subprocess.run(vcmd, check=True)

# =====================  Task 6 – Validation against truth  =====================
# Author: you  (2025-xx-xx)
# ------------------------------------------------------------------------------

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import scipy.io as sio                              # only if you store .mat


def validate_against_truth(
        truth_path      = Path('STATE_X001.txt'),
        est_mat_path    = Path('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat'),
        results_dir     = Path('results'),
        plot_fname      = 'Task6_Validation_TRIAD.pdf',
        time_key        = 'time_residuals',
        pos_key         = 'fused_pos',
        vel_key         = 'fused_vel',
        quat_key        = 'attitude_q',            # (body→ECEF, scalar-first)
        P_key           = 'P_hist'
):
    """Compare EKF estimates with ground truth and verify 3-sigma consistency."""
    # -------------------------------------------------------------------------
    # 1. load ground truth  ---------------------------------------------------
    truth_cols = ['count','time',
                  'X','Y','Z',
                  'Vx','Vy','Vz',
                  'q0','q1','q2','q3']
    truth = pd.read_csv(truth_path, sep=r'\s+', names=truth_cols, comment='#')
    t_truth = truth['time'].values
    pos_truth = truth[['X','Y','Z']].values
    vel_truth = truth[['Vx','Vy','Vz']].values
    quat_truth = truth[['q0','q1','q2','q3']].values

    # -------------------------------------------------------------------------
    # 2. load estimator output  ----------------------------------------------
    est = sio.loadmat(est_mat_path)
    t_est  = est[time_key].ravel()
    pos_est = est[pos_key]
    vel_est = est[vel_key]
    quat_est = est[quat_key]                     # shape (N,4)
    P_diag = None
    if P_key in est:
        P_diag = np.diagonal(est[P_key], axis1=1, axis2=2)

    # -------------------------------------------------------------------------
    # 3. time-sync by interpolation  ------------------------------------------
    def interp(arr, t_arr, t_target):
        """interpolate each column independently"""
        return np.vstack([np.interp(t_target, t_arr, arr[:, i]) for i in range(arr.shape[1])]).T

    pos_est_i  = interp(pos_est,  t_est, t_truth)
    vel_est_i  = interp(vel_est,  t_est, t_truth)
    quat_est_i = interp(quat_est, t_est, t_truth)    # SLERP is nicer – linear ok for small Δt
    P_i = None
    if P_diag is not None:
        P_i = interp(P_diag, t_est, t_truth)    # 1-d interpolation of every σ²

    # -------------------------------------------------------------------------
    # 4. compute errors  ------------------------------------------------------
    err_pos = pos_est_i - pos_truth
    err_vel = vel_est_i - vel_truth

    # quaternion attitude error (small-angle approx)
    def quat_err(q_hat, q_true):
        """q_err = q_true⁻¹ ⊗ q_hat  ->  convert to angle (deg)"""
        r_hat  = R.from_quat(q_hat[:,1:4])
        r_true = R.from_quat(q_true[:,1:4])
        q_err  = r_true.inv() * r_hat
        ang    = q_err.magnitude() * 180/np.pi
        return ang

    err_att = quat_err(quat_est_i, quat_truth)

    # -------------------------------------------------------------------------
    # 5. 3-sigma envelopes  ---------------------------------------------------
    sigma_pos = sigma_vel = sigma_att = None
    if P_i is not None:
        sigma_pos = np.sqrt(P_i[:, 0:3])
        if P_i.shape[1] >= 6:
            sigma_vel = np.sqrt(P_i[:, 3:6])
        if P_i.shape[1] > 9:
            sigma_att = np.sqrt(P_i[:, 9]) * 180/np.pi  # attitude error state

    # -------------------------------------------------------------------------
    # 6. plots  ---------------------------------------------------------------
    fig, ax = plt.subplots(3,1, figsize=(8.5,10), sharex=True)

    # Position ---------------------------------------------------------------
    ax[0].plot(t_truth, err_pos)
    if sigma_pos is not None:
        ax[0].plot(t_truth,  3 * sigma_pos, 'k--', alpha=.6)
        ax[0].plot(t_truth, -3 * sigma_pos, 'k--', alpha=.6)
        ax[0].legend(['dX', 'dY', 'dZ', '±3σ'])
    else:
        ax[0].legend(['dX', 'dY', 'dZ'])
    ax[0].set_ylabel('pos err [m]')

    # Velocity ---------------------------------------------------------------
    ax[1].plot(t_truth, err_vel)
    if sigma_vel is not None:
        ax[1].plot(t_truth, 3 * sigma_vel, 'k--', alpha=.6)
        ax[1].plot(t_truth, -3 * sigma_vel, 'k--', alpha=.6)
        ax[1].legend(['dVx', 'dVy', 'dVz', '±3σ'])
    else:
        ax[1].legend(['dVx', 'dVy', 'dVz'])
    ax[1].set_ylabel('vel err [m/s]')

    # Attitude ---------------------------------------------------------------
    ax[2].plot(t_truth, err_att, label='angle error')
    if sigma_att is not None:
        ax[2].plot(t_truth, 3 * sigma_att, 'k--', label='±3σ')
        ax[2].plot(t_truth, -3 * sigma_att, 'k--')
        ax[2].legend()
    else:
        ax[2].legend(['angle error'])
    ax[2].set_ylabel('att. err [deg]')
    ax[2].set_xlabel('time [s]')

    fig.suptitle('Task 6 – Consistency check vs. STATE_X001 truth')
    fig.tight_layout()

    out = results_dir/plot_fname
    fig.savefig(out, dpi=300)
    print(f'Validation plot written to  {out}')

    # -------------------------------------------------------------------------
    # 7. quick stats  ---------------------------------------------------------
    print('\n=== RMSE wrt truth ===')
    for name, e in zip(['pos [m]','vel [m/s]','att [deg]'],
                       [err_pos,   err_vel,   err_att]):
        rms = np.sqrt(np.mean(e**2, axis=0)) if e.ndim==2 else np.sqrt(np.mean(e**2))
        print(f'{name:10s}: {rms}')


if __name__ == '__main__':
    validate_against_truth(
        truth_path   = Path('STATE_X001.txt'),
        est_mat_path = Path('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat')
    )


