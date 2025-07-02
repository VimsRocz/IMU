#!/usr/bin/env bash
# Populate the local vendor/ folder with all dependencies required for
# offline installs. Binary wheels are downloaded whenever possible and
# source archives are fetched for packages that do not provide wheels.

set -euo pipefail

mkdir -p vendor
echo "\n📦 Downloading binary wheels for main requirements..."
pip download --only-binary=:all: --dest vendor -r requirements.txt

echo "\n📦 Downloading wheel for flake8 (used in CI)..."
pip download --only-binary=:all: --dest vendor flake8

echo "\n📦 Downloading source distributions for packages without wheels..."
pip download --no-binary=filterpy,fpdf,geomag --dest vendor filterpy fpdf geomag

echo "\n✅ vendor/ directory is ready"
