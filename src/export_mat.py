#!/usr/bin/env python3
"""Export results for MATLAB"""
import os
import sys
import numpy as np
from scipy.io import savemat
import pathlib

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

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
    out = {
        'x_est': data['x_est'],
        'P': data['P'],
        'residuals': data['residuals'],
    }
    if 'fused_pos' in data:
        out['fused_pos'] = data['fused_pos']
    if 'fused_vel' in data:
        out['fused_vel'] = data['fused_vel']
    savemat(mat_file, out)
    print(f'Exported {mat_file}')
