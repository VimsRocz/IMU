#!/usr/bin/env bash
set -euo pipefail

# Install Python packages needed for running the tests
pip install --upgrade pip setuptools wheel build
pip install -r requirements.txt -r requirements-dev.txt
