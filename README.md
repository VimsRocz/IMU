# IMU_GNSS_Fusion


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

The repository bundles three IMU logs (`IMU_X001.dat`–`IMU_X003.dat`) and two GNSS traces (`GNSS_X001.csv`, `GNSS_X002.csv`) under the `Data/` folder.
Note that `IMU_X003.dat` pairs with `GNSS_X002.csv`; there is no separate `GNSS_X003.csv`.

If your data resides in a different location, set the environment variable
`IMU_DATA_PATH` to point to that directory and the helper scripts will search it
before falling back to the bundled `Data/` folder.
Multiple paths can be specified using the platform's path separator (``:`` on
Unix).
For example:

```bash
export IMU_DATA_PATH=/path/to/my/datasets
```

Process all datasets with every attitude initialisation method using

```bash
python Python/run_all_methods.py --config config_small.yml
```

The `config_small.yml` file lists the dataset pairs and can be customised
for new recordings.  Each entry specifies an ``imu`` and a ``gnss`` file
name and an optional ``methods`` list to restrict processing.

To add your own data simply append another mapping under ``datasets`` with
the file names of your log pair and list any additional methods under the
``methods`` key.

To run only the TRIAD method use the helper script

```bash
python Python/run_triad_only.py
```

Both scripts write their results to `Python/results/` and, when a reference trajectory is available, automatically validate the output.

## Running the TRIAD pipeline in MATLAB
Execute `TRIAD.m` from the `MATLAB` folder to run Tasks 1–5 on the bundled
data. The script prints the same log messages as the Python version and validates
the results automatically.

## MATLAB usage

A MATLAB implementation of the same pipeline lives under `MATLAB/`. All example
datasets reside in the top-level `Data/` folder, and the plotting scripts write
their PDFs to `MATLAB/results/`. See [MATLAB/README.md](MATLAB/README.md) for
instructions on running `main.m` and the individual task scripts.

## Further documentation

Additional developer notes and troubleshooting tips have moved to [docs/DeveloperNotes.md](docs/DeveloperNotes.md). The `docs/` folder also contains debugging guides and plotting examples.
