#!/usr/bin/env python3
"""Export results for MATLAB"""
import os
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

import numpy as np
from scipy.io import savemat
import pathlib

if not os.path.exists('STATE_X001.txt'):
    print("ERROR: export_mat.py must be run from its own folder or with full path.")
    sys.exit(1)

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
