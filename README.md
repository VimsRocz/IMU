# IMU_GNSS_Fusion
IMU data processing and initialization tools (Python)

### Table of Contents
- [Installation](#installation)
- [Running the Pipeline](#running-the-pipeline)
  - [run_all_datasets.py](#run_all_datasetspy)
  - [run_triad_only.py](#run_triad_onlypy)
  - [GNSS_IMU_Fusion_single](#gnss_imu_fusion_singleimu_file-gnss_file)
- [Per-Task Overview](#per-task-overview)
- [Datasets](#datasets)
- [Running validation](#running-validation)
- [Tests](#tests)
- [Documentation](#documentation)
- [Appendix: Advanced Topics](#appendix-advanced-topics)
### Installation

`src/run_all_datasets.py` installs the required packages automatically the first
time you run it. If you'd rather set them up beforehand, install the
dependencies manually with:

```bash
pip install -r requirements.txt
```

For a minimal setup you can also install the packages individually:

```bash
pip install numpy matplotlib scipy filterpy
```

The tests, however, require **all** packages from `requirements.txt` *and* the
development extras listed in `requirements-dev.txt`.  This pulls in heavier
libraries such as `cartopy`.  Installing them in a virtual environment or
container keeps your base Python setup clean.  Running `make test` installs
both requirement files automatically before executing `pytest`.

If you run into issues with filterpy on Ubuntu:

```bash
pip install --upgrade pip setuptools
pip install filterpy
```

#### Installing FilterPy on Ubuntu

If installation of `filterpy` fails (for example due to missing build tools) run:

```bash
sudo apt update && sudo apt install python3-pip
pip3 install filterpy
```

#### Installing test requirements

To run the unit tests you need `numpy`, `pandas`, `scipy` and `cartopy` which are all included in `requirements.txt`. Install them together with `pytest` via:

```bash
pip install -r requirements-dev.txt -r requirements.txt
```
Both requirement files **must** be installed before executing `pytest`.
`pytest` itself is only listed in `requirements-dev.txt`, so skipping that
file will leave the `pytest` command unavailable.

#### Offline installation

If your CI environment cannot reach PyPI, populate a local `vendor/`
directory with all required packages by running:

```bash
scripts/download_vendor.sh
```

The script downloads binary wheels whenever possible and falls back to
source archives for packages that do not provide wheels. The CI
workflow installs from `vendor/` using `--no-index`.


If the build error complains about Cython install it explicitly first:

```bash
pip3 install cython
pip3 install filterpy
```

### Per-Task Overview

The five pipeline tasks roughly correspond to the steps of the MATLAB
implementation. Each task produces intermediate results that feed into the
next one:

* **Task&nbsp;1 &ndash; Reference Vectors:** defines gravity and Earth-rotation
  vectors from the first GNSS fix. [Details](docs/TRIAD_Task1_Wiki.md)
* **Task&nbsp;2 &ndash; Body Vectors:** measures gravity and Earth-rate in the
  IMU frame and estimates sensor biases. [Details](docs/TRIAD_Task2_Wiki.md)
* **Task&nbsp;3 &ndash; Initial Attitude:** solves Wahba's problem with the
  TRIAD algorithm. [Details](docs/TRIAD_Task3_Wiki.md)
* **Task&nbsp;4 &ndash; IMU-Only Integration:** integrates the corrected IMU
  data and compares the result with GNSS. [Details](docs/TRIAD_Task4_Wiki.md)
* **Task&nbsp;5 &ndash; Kalman Fusion:** fuses IMU and GNSS with a simple
  Kalman filter to produce the final trajectory. [Details](docs/TRIAD_Task5_Wiki.md)

### Datasets

The repository includes three IMU logs and two GNSS traces:

* `IMU_X001.dat` with `GNSS_X001.csv`
* `IMU_X002.dat` with `GNSS_X002.csv`
* `IMU_X003.dat` with `GNSS_X002.csv` (no dedicated GNSS log was recorded)

For quick tests the repository also provides truncated versions of each
file:

* `IMU_X001_small.dat`, `IMU_X002_small.dat`, `IMU_X003_small.dat`
  contain the first 1 000 IMU samples (about 2.5 s each)
* `GNSS_X001_small.csv`, `GNSS_X002_small.csv` contain the first ten GNSS
  epochs
* `STATE_X001_small.txt` holds the first 100 reference states

These mini logs drastically reduce runtimes when validating the pipeline or
the MATLAB scripts.

### Running validation

To process the bundled datasets using only the TRIAD initialisation and
validate the results, run either the Python or MATLAB helper script:

```bash
python src/run_triad_only.py
```

```matlab
run_triad_only
```

All the batch scripts (for example `src/run_triad_only.py` and
`src/run_all_datasets.py`) create a `results/` folder in the directory you
launch them from. All output files are written to this directory. When a
matching ground truth file such as `STATE_X001.txt` is available the script
automatically calls `validate_with_truth.py` to compare the estimated
trajectory against it. The validation summary and overlay plots are saved next
to the exported `.mat` files. `validate_with_truth.py` now produces
`<method>_<frame>_overlay_truth.pdf` files that overlay the fused trajectory
with the reference in the NED, ECEF and body frames. The input data files are
looked up relative to the repository root, so you can run the scripts from any
directory.
### Sample Processing Report

A sample run of `run_triad_only.py` is documented in [Report/](Report/index.md). Each page lists the equations and the PDF figures generated in the `results/` directory.

Typical result PDFs:

- `location_map.pdf` – initial location map
- `task3_errors_comparison.pdf` – attitude initialisation error comparison
- `task3_quaternions_comparison.pdf` – quaternion components for initialisation
- `task4_comparison_ned.pdf` – GNSS vs IMU in NED frame
- `task4_mixed_frames.pdf` – GNSS/IMU data in mixed frames
- `task4_all_ned.pdf` – integrated data in NED frame
- `task4_all_ecef.pdf` – integrated data in ECEF frame
- `task4_all_body.pdf` – integrated data in body frame
- `task5_results_<method>.pdf` – Kalman filter results for each method
- `task5_all_ned.pdf` – Kalman filter results in NED frame
- `task5_all_ecef.pdf` – Kalman filter results in ECEF frame
- `task5_all_body.pdf` – Kalman filter results in body frame
- `<method>_residuals.pdf` – position and velocity residuals
- `<method>_attitude_angles.pdf` – attitude angles over time
- `*_overlay_truth.pdf` – overlay of fused output with ground truth when available

### Notes

`src/GNSS_IMU_Fusion.py` now detects a low-motion interval in the IMU data to
estimate the initial accelerometer and gyroscope biases. The magnitude of the
static accelerometer vector is used to compute a simple scale factor so that the
measured gravity is close to 9.81 m/s². This improves the attitude
initialisation when the first samples are not perfectly static or the sensor
scale is slightly off.
### Verifying Earth Rotation Rate

Run any dataset with the `--verbose` flag to print the **Earth rotation magnitude** measured from the static gyroscope data.
It should be close to `7.29e-5` rad/s. You can compute the nominal value with:

```bash
python - <<"EOF"
import numpy as np; print(2*np.pi/86400)
EOF
```

If the measured magnitude differs by more than a few percent, the IMU may not be completely still or needs calibration.


### Running the Pipeline

#### run_all_methods.py

Use `src/run_all_methods.py` to execute the fusion script with the TRIAD,
Davenport and SVD initialisation methods in sequence.  Provide a YAML
configuration file to specify the datasets:

```bash
python src/run_all_methods.py --config your_config.yml
```

Running the script without `--config` processes the bundled example data sets.


#### run_all_datasets.py

To process every IMU/GNSS pair defined in `src/run_all_datasets.py`, simply run:

```bash
python src/run_all_datasets.py
```
By default this processes each dataset with the TRIAD, Davenport and SVD
initialisation methods. To limit the run to a single method pass
`--method METHODNAME`.
The script shows a progress bar and finishes with a small summary table:

```
All cases: 100%|##########| 9/9 [00:12<00:00,  1.31s/it]
┏━━━━━━━━━┳━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━┓
┃ Method  ┃ RMSE (m) ┃ Final Error (m) ┃ Runtime (s) ┃
┡━━━━━━━━━╇━━━━━━━━━━╇━━━━━━━━━━━━━━╇━━━━━━━━━━━┩
┃ TRIAD   ┃  12.4    ┃  8.7         ┃   1.2      ┃
┃ ...     ┃  ...     ┃  ...         ┃   ...      ┃
└─────────┴──────────┴──────────────┴───────────┘
```

#### run_triad_only.py

If you want to process all datasets using just the TRIAD initialisation method, run the helper script `src/run_triad_only.py` or the MATLAB script `run_triad_only.m`:

```bash
python src/run_triad_only.py
```
```matlab
run_triad_only
```
This is equivalent to running `python src/run_all_datasets.py --method TRIAD`.


After all runs complete you can compare the datasets side by side:

```bash
python src/plot_compare_all.py
```
This creates one `all_datasets_<method>_comparison.pdf` per method in
the `results/` directory you ran the script from.

#### GNSS_IMU_Fusion_single(imu_file, gnss_file)

[`GNSS_IMU_Fusion_single`](MATLAB/GNSS_IMU_Fusion_single.m) is now a
self-contained script that implements the full Task&nbsp;1&ndash;Task&nbsp;5
pipeline internally and writes the resulting plots to `results/`.  The
stand‑alone `Task_*.m` files remain in the repository for workflows that call
them individually:

```matlab
GNSS_IMU_Fusion_single('IMU_X001.dat', 'GNSS_X001.csv')
```

#### FINAL.py

Run `src/FINAL.py` to process just the first IMU/GNSS pair with all three
initialisation methods.  The script calls the fusion routine directly, so no
additional wrapper scripts are involved:

```bash
python src/FINAL.py
```
This produces the same logs and summary table as `src/run_all_datasets.py` but
only for `IMU_X001.dat` and `GNSS_X001.csv`. You can still pass `--method` or a
YAML config file just like with the batch runner.
If your shell complains about `event not found` when you try running the
shebang line directly, simply invoke the script with `python src/FINAL.py` or run it
as `./src/FINAL.py` instead.

### Automated figure generation

The module `auto_plots.py` contains helper routines for creating the standard
set of plots and a summary table of metrics for each batch run.  Integrate
these helpers into your own batch script to automatically export the six
"must‑have" figures (tasks 3–5 and validation plots) and a CSV/LaTeX table of
RMSE and innovation statistics.

```python
from auto_plots import run_batch

# one IMU/GNSS pair and one method for brevity
datasets = [("IMU_X001.dat", "GNSS_X001.csv")]
methods = ["TRIAD"]
run_batch(datasets, methods)
```

The snippet above loads the files using :func:`load_data` and places the
generated figures in `results/auto_plots/`.

Interactive exploration lives in the `notebooks/` folder. Open
`notebooks/demo.ipynb` to try the plotting utilities in Jupyter.
Additional plotting examples and a results template are available in [docs/PlottingExamples.md](docs/PlottingExamples.md).

A codex-style plotting checklist is available in [docs/PlottingChecklist.md](docs/PlottingChecklist.md).

### Summary CSV format

`summarise_runs.py` parses the log files produced by the batch scripts and
writes `results/summary.csv`. Each row contains:

- `method` – attitude initialisation method name
- `imu` – IMU data file
- `gnss` – GNSS data file
- `rmse_pos` – overall position RMSE in metres
- `final_pos` – final position error in metres

`generate_summary.py` reads this CSV to build the PDF report.

### Tests

Run the unit tests with `pytest`. **Installing the required Python packages is
mandatory** before executing any tests. The suite relies on *all* entries in
`requirements.txt` as well as the development dependencies in
`requirements-dev.txt` – including heavier libraries such as `cartopy` that can
take some time to build. Using a dedicated virtual environment or container is
strongly recommended.  The `test` target in the `Makefile` installs both
requirement files for you. Alternatively, run the helper script
`scripts/setup_tests.sh` before invoking `pytest`:

```bash
./scripts/setup_tests.sh
pytest -q
```

You can also simply run `make test` to install both requirement files and
execute the test suite in one command.

### Linting

Static checks run with [Ruff](https://github.com/astral-sh/ruff):

```bash
ruff check src tests
```

Rule `E402` is ignored in `pyproject.toml` as some modules rely on runtime
imports.

### Additional scripts

The repository contains a few helper scripts that are not part of the regular
pipeline but can be handy for quick experiments:

- `fusion_single.py` &ndash; early stand‑alone prototype of the IMU&ndash;GNSS
  fusion routine. It exposes the full bias estimation and Kalman logic in a
  single file and is useful for debugging or exploring tuning options.
- `validate_filter.py` &ndash; command&ndash;line tool to compare a saved filter
  state history against GNSS measurements and plot the residuals.

These utilities are optional and not exercised by the unit tests.

### Next Steps

- **Logging:** Extend the built-in `logging` with the `rich` console handler to
  get colourful status messages during long runs.
- **Documentation:** The devcontainer includes Sphinx and MkDocs. Generate the
  API docs with `sphinx-build` and publish user guides with `mkdocs`.
- **CI:** Set up a simple GitHub Actions workflow that installs
  `requirements.txt`, runs `flake8` and the `pytest` suite on every pull
  request.
- **Debugging:** If the MATLAB pipeline produces kilometre-scale drift,
  consult `docs/DebuggingDrift.md` for troubleshooting steps.
- **Security:** Disable GitHub's default CodeQL scanner as described in
  [docs/disable-default-codeql.md](docs/disable-default-codeql.md).
### Appendix: Advanced Topics

See [docs/AdvancedTopics.md](docs/AdvancedTopics.md) for GitHub Codespaces instructions and MATLAB usage.

### Documentation

- [docs/TRIAD_Task1_Wiki.md](docs/TRIAD_Task1_Wiki.md) – defining reference vectors
- [docs/TRIAD_Task2_Wiki.md](docs/TRIAD_Task2_Wiki.md) – measuring body-frame vectors
- [docs/TRIAD_Task3_Wiki.md](docs/TRIAD_Task3_Wiki.md) – initial attitude determination
- [docs/TRIAD_Task4_Wiki.md](docs/TRIAD_Task4_Wiki.md) – GNSS/IMU integration
- [docs/TRIAD_Task5_Wiki.md](docs/TRIAD_Task5_Wiki.md) – Kalman filter fusion

- [Report/](Report/index.md) – summary of a typical run
See [docs/CHANGELOG.md](docs/CHANGELOG.md) for a list of recent features.
