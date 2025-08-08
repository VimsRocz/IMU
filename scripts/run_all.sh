#!/usr/bin/env bash
# Usage: scripts/run_all.sh
set -euo pipefail

proj_dir="$(cd "$(dirname "$0")"/.. && pwd)"
cd "$proj_dir"

echo "Project: $proj_dir"

# Show inputs (helps detect path mistakes fast)
ls -l IMU_X002.dat GNSS_X002.csv STATE_IMU_X001.txt || true

# MATLAB (no desktop)
matlab -batch "addpath('MATLAB'); run_triad_only(struct('dataset_id','X002','method','TRIAD')); exit"

# Python (totally independent)
python3 src/run_triad_only.py
