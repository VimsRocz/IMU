import argparse
import os

import numpy as np
import pandas as pd
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


def load_estimate(path):
    """Return trajectory, quaternion and covariance from an NPZ or MAT file."""

    def _maybe(keys, container, default=None):
        for k in keys:
            if k in container:
                return container[k]
        return default

    if path.endswith(".npz"):
        data = np.load(path, allow_pickle=True)
        t = _maybe(["time_residuals", "time"], data)
        if t is not None:
            t = np.asarray(t).squeeze()
        pos = _maybe(["pos_ned", "pos"], data)
        vel = _maybe(["vel_ned", "vel"], data)
        quat = _maybe(["quat_log", "quat"], data)
        if quat is None and "euler" in data:
            quat = R.from_euler("xyz", data["euler"], degrees=True).as_quat()
            quat = quat[:, [3, 0, 1, 2]]
        if pos is None and "residual_pos" in data and "innov_pos" in data:
            pos = data["residual_pos"] + data["innov_pos"]
        if vel is None and "residual_vel" in data and "innov_vel" in data:
            vel = data["residual_vel"] + data["innov_vel"]
        est = {
            "time": t,
            "pos": pos,
            "vel": vel,
            "quat": quat,
            "P": data.get("P"),
        }
    else:
        m = loadmat(path)
        t = _maybe(["time_residuals", "time"], m)
        if t is not None:
            t = np.asarray(t).squeeze()
        pos = _maybe(["pos_ned", "pos"], m)
        vel = _maybe(["vel_ned", "vel"], m)
        quat = _maybe(["quat_log", "quat"], m)
        if quat is None and "euler" in m:
            quat = R.from_euler("xyz", m["euler"], degrees=True).as_quat()
            quat = quat[:, [3, 0, 1, 2]]
        if pos is None and "residual_pos" in m and "innov_pos" in m:
            pos = m["residual_pos"] + m["innov_pos"]
        if vel is None and "residual_vel" in m and "innov_vel" in m:
            vel = m["residual_vel"] + m["innov_vel"]
        est = {
            "time": t,
            "pos": pos,
            "vel": vel,
            "quat": quat,
            "P": m.get("P"),
        }

    if est["time"] is None:
        try:
            est["time"] = np.loadtxt("STATE_X001.txt", comments="#", usecols=1)[
                : len(est["pos"])
            ]
        except OSError:
            est["time"] = np.arange(len(est["pos"]))

    return est


def main():
    ap = argparse.ArgumentParser(
        description="Compare filter output with ground truth and plot error "
                    "curves with optional ±3σ bounds.")
    ap.add_argument('--est-file', required=True, help='NPZ or MAT file with filter results')
    ap.add_argument('--truth-file', required=True, help='CSV of true state (STATE_X001.txt)')
    ap.add_argument('--output', default='results', help='directory for the generated PDFs')
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)

    est = load_estimate(args.est_file)
    truth = np.loadtxt(args.truth_file)
    n = min(len(est['pos']), truth.shape[0])
    err_pos = est['pos'][:n] - truth[:n, :3]
    err_vel = None
    err_quat = None

    if est.get('vel') is not None:
        err_vel = est['vel'][:n] - truth[:n, 5:8]

    if est.get('quat') is not None:
        q_true = truth[:n, 8:12]
        q_est = est['quat'][:n]
        r_true = R.from_quat(q_true[:, [1, 2, 3, 0]])
        r_est = R.from_quat(q_est[:, [1, 2, 3, 0]])
        r_err = r_est * r_true.inv()
        err_quat = r_err.as_quat()[:, [3, 0, 1, 2]]

    sigma_pos = sigma_vel = sigma_quat = None
    if est['P'] is not None:
        diag = np.diagonal(est['P'], axis1=1, axis2=2)[:n]
        if diag.shape[1] >= 3:
            sigma_pos = 3 * np.sqrt(diag[:, :3])
        if diag.shape[1] >= 6:
            sigma_vel = 3 * np.sqrt(diag[:, 3:6])
        if diag.shape[1] >= 10:
            sigma_quat = 3 * np.sqrt(diag[:, 6:10])

    def plot_err(t, err, sigma, labels, prefix):
        for i, lbl in enumerate(labels):
            plt.figure()
            plt.plot(t, err[:, i], label='error')
            if sigma is not None:
                plt.plot(t, sigma[: len(t), i], 'r--', label='+3σ')
                plt.plot(t, -sigma[: len(t), i], 'r--')
            plt.xlabel('Time [s]')
            plt.ylabel(f'{lbl} error')
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(args.output, f'{prefix}_{lbl}.pdf'))
            plt.close()

    t = est['time'][:n]
    plot_err(t, err_pos, sigma_pos, ['X', 'Y', 'Z'], 'pos_err')
    if err_vel is not None:
        plot_err(t, err_vel, sigma_vel, ['Vx', 'Vy', 'Vz'], 'vel_err')
    if err_quat is not None:
        plot_err(t, err_quat, sigma_quat, ['q0', 'q1', 'q2', 'q3'], 'att_err')


if __name__ == '__main__':
    main()
