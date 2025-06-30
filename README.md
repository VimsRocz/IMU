# IMU_GNSS_Fusion
[![CI](https://github.com/VimsRocz/IMU/actions/workflows/ci.yml/badge.svg)](https://github.com/VimsRocz/IMU/actions/workflows/ci.yml)

Python and MATLAB tools for initialising and validating IMU+GNSS trajectories.

## Installation

Install the required Python packages with

```bash
pip install -r requirements.txt
```

During development you can also install the project itself in editable mode:

```bash
pip install -e .
```

## Running the Python pipeline

The repository bundles three IMU logs (`IMU_X001.dat`–`IMU_X003.dat`) and two GNSS traces (`GNSS_X001.csv`, `GNSS_X002.csv`).

Process all datasets with every attitude initialisation method using

```bash
python run_all_datasets.py
```

To run only the TRIAD method use the helper script

```bash
python run_triad_only.py
```

Both scripts write their results to `results/` and, when a reference trajectory is available, automatically validate the output.

## MATLAB usage

A MATLAB implementation of the same pipeline lives under `MATLAB/`. See [MATLAB/README.md](MATLAB/README.md) for instructions on running `main.m` and the individual task scripts.

## Further documentation

Additional developer notes and troubleshooting tips have moved to [docs/DeveloperNotes.md](docs/DeveloperNotes.md). The `docs/` folder also contains debugging guides and plotting examples.
