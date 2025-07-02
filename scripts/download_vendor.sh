#!/usr/bin/env bash
set -euo pipefail
mkdir -p vendor
pip download --only-binary=:all: --dest vendor -r requirements.txt
