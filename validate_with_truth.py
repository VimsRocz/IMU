import argparse
import numpy as np
import pandas as pd
from scipy.io import loadmat
import matplotlib.pyplot as plt


def load_estimate(path, truth_len=None):
    """Return position estimates and time vector from an NPZ or MAT file."""

    if path.endswith(".npz"):
        data = np.load(path, allow_pickle=True)
        t = data.get("time_residuals")
        if t is not None:
            t = np.asarray(t).squeeze()
        est = {
            "time": t,
            "pos": data["residual_pos"] + data["innov_pos"],
            "P": None,
        }
    else:
        m = loadmat(path)
        t = m.get("time_residuals")
        if t is not None:
            t = np.asarray(t).squeeze()
        est = {
            "time": t,
            "pos": m["residual_pos"] + m["innov_pos"],
            "P": m.get("P"),
        }

    if est["time"] is None:
        try:
            time_vec = np.loadtxt("STATE_X001.txt", comments="#", usecols=1)
            n = truth_len if truth_len is not None else len(est["pos"])
            est["time"] = time_vec[:n]
        except OSError:
            n = truth_len if truth_len is not None else len(est["pos"])
            est["time"] = np.arange(n)

    return est


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--est-file', required=True)
    ap.add_argument('--truth-file', required=True)
    ap.add_argument('--output', default='results')
    args = ap.parse_args()

    # Load reference state: columns 2-4 are the true ECEF position.
    truth_pos = np.loadtxt(args.truth_file, comments="#", usecols=(2, 3, 4))

    # Optional velocity and attitude columns (5-7 and 8-11) may be present.
    try:
        truth_vel = np.loadtxt(args.truth_file, comments="#", usecols=(5, 6, 7))
    except Exception:
        truth_vel = None
    try:
        truth_att = np.loadtxt(args.truth_file, comments="#", usecols=(8, 9, 10, 11))
    except Exception:
        truth_att = None

    est = load_estimate(args.est_file, truth_len=len(truth_pos))
    err = est["pos"][: len(truth_pos)] - truth_pos

    if est['P'] is not None:
        sigma = 3 * np.sqrt(np.diagonal(est['P'], axis1=1, axis2=2)[:, :3])
    else:
        sigma = None

    t = est['time'][: err.shape[0]]
    labels = ['X', 'Y', 'Z']
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
        plt.savefig(f"{args.output}/error_{lbl}.pdf")
        plt.close()


if __name__ == '__main__':
    main()
