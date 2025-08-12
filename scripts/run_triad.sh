#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if command -v matlab >/dev/null 2>&1; then
  echo "Running TRIAD via MATLAB batch..."
  matlab -batch "run('MATLAB/run_triad_batch.m')"
else
  echo "MATLAB is not available in PATH. Please run inside MATLAB:"
  echo "  addpath('MATLAB'); addpath('MATLAB/src'); run_triad_only();"
  exit 1
fi
