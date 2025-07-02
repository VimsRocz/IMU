#!/usr/bin/env bash
set -euo pipefail

# Install Python packages needed for running the tests
pip install -r requirements.txt -r requirements-dev.txt
