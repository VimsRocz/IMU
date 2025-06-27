import argparse
import numpy as np
import pandas as pd
from scipy.io import loadmat
import matplotlib.pyplot as plt


def load_estimate(path):
    if path.endswith('.npz'):
        data = np.load(path)
        est = {
            'time': np.arange(len(data['euler'])),
            'pos': data['residual_pos'] + data['innov_pos'],
            'P': None,
        }
    else:
        m = loadmat(path)
        est = {
            'time': np.arange(m['euler'].shape[0]),
            'pos': m['residual_pos'] + m['innov_pos'],
            'P': m.get('P'),
        }
    return est


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--est-file', required=True)
    ap.add_argument('--truth-file', required=True)
    ap.add_argument('--output', default='results')
    args = ap.parse_args()

    est = load_estimate(args.est_file)
    truth = np.loadtxt(args.truth_file)
    err = est['pos'] - truth[:, :3]

    if est['P'] is not None:
        sigma = 3 * np.sqrt(np.diagonal(est['P'], axis1=1, axis2=2)[:, :3])
    else:
        sigma = None

    t = est['time']
    labels = ['X', 'Y', 'Z']
    for i, lbl in enumerate(labels):
        plt.figure()
        plt.plot(t, err[:, i], label='error')
        if sigma is not None:
            plt.plot(t, sigma[:, i], 'r--', label='+3σ')
            plt.plot(t, -sigma[:, i], 'r--')
        plt.xlabel('Time [s]')
        plt.ylabel(f'{lbl} error')
        plt.legend()
        plt.tight_layout()
        plt.savefig(f"{args.output}/error_{lbl}.pdf")
        plt.close()


if __name__ == '__main__':
    main()
