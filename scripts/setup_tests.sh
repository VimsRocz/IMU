#!/usr/bin/env bash
set -euo pipefail

# Install Python packages needed for running the tests. The `[tests]` extras
# defined in `pyproject.toml` pull in the full runtime stack along with
# additional test dependencies such as `numpy`, `scipy`, `matplotlib`,
# `filterpy`, `cartopy` and `pytest`.
pip install --upgrade pip setuptools wheel build
pip install -e .[tests]
