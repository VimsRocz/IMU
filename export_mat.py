#!/usr/bin/env python3
"""Export results for MATLAB"""
import numpy as np
from scipy.io import savemat
import pathlib

DATASETS = ['X001', 'X002', 'X003']

for tag in DATASETS:
    npz_file = pathlib.Path(f'results_{tag}.npz')
    if not npz_file.exists():
        print(f'Skipping {npz_file} (not found)')
        continue
    data = np.load(npz_file)
    mat_file = f'results_{tag}.mat'
    savemat(mat_file, {
        'x_est': data['x_est'],
        'P': data['P'],
        'residuals': data['residuals'],
    })
    print(f'Exported {mat_file}')
