# IMU_GNSS_Fusion
IMU data processing and initialization tools (Python)

## Installation

`src/run_all_datasets.py` installs the required packages automatically the first
time you run it. If you'd rather set them up beforehand, install the
dependencies manually with:

```bash
pip install -r requirements.txt
```

For a minimal setup you can also install the packages individually:

```bash
pip install numpy matplotlib filterpy
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

### Installing FilterPy on Ubuntu

If installation of `filterpy` fails (for example due to missing build tools) run:

```bash
sudo apt update && sudo apt install python3-pip
pip3 install filterpy
```

### Installing test requirements

To run the unit tests you need `numpy`, `pandas`, `scipy` and `cartopy` which are all included in `requirements.txt`. Install them together with `pytest` via:

```bash
pip install -r requirements-dev.txt -r requirements.txt
```
Both requirement files **must** be installed before executing `pytest`.
`pytest` itself is only listed in `requirements-dev.txt`, so skipping that
file will leave the `pytest` command unavailable.


If the build error complains about Cython install it explicitly first:

```bash
pip3 install cython
pip3 install filterpy
```

## Datasets

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

## Running validation

To process the bundled datasets using only the TRIAD initialisation and
validate the results, run either the Python or MATLAB helper script:

```bash
python src/run_triad_only.py
```

```matlab
run_triad_only
```

All the batch scripts (for example `src/run_triad_only.py` and
`src/run_all_datasets.py`) create a `results/` folder in the directory you launch them from. All output files are written to this directory. When a matching ground truth file such as `STATE_X001.txt` is available the script
automatically calls `validate_with_truth.py` to compare the estimated trajectory
against it. The validation summary and plots are saved alongside the exported
`.mat` files in `results/`.
The input data files are looked up relative to the repository root, so you can
run the scripts from any directory.

## 🚀 Developing & Debugging in GitHub Codespaces

1. **Open in Codespace**  
   Click **Code → Open with Codespaces** on the repo.

2. **Install dependencies**  
   Codespaces will automatically build the container (per `.devcontainer/`), install Python & your `requirements.txt`.

3. **Enable verbose diagnostics**
   We’ve added a `--verbose` flag to `src/run_all_datasets.py` that enables all the extra tables and timing you requested. `src/GNSS_IMU_Fusion.py` also accepts `--verbose` for the same detailed diagnostics.

4. **Run with diagnostics**  
   In the integrated terminal:
   ```bash
   python src/run_all_datasets.py --verbose 2>&1 | tee debug_output.log
   ```

This will print:
	•	Static-Calibration Summary

=== IMU Calibration ===
Accel bias [m/s²]:    [ 0.5687, -6.9884,  0.9356 ]
Gyro bias  [rad/s]:   [ -1e-08, 6.6e-09, -3.1e-09 ]
Accel scale factor:   1.0000
Static interval:      samples 234–634 (Δt=1 s)

	•	Attitude-Determination Diagnostics for each method (gravity/Earth-rate errors, condition numbers, degeneracies)
	•	GNSS Quality Metrics (HDOP/PDOP statistics, satellite counts)
	•	ZUPT & KF events summary (counts, total ZUPT time, final covariance)
	•	Trajectory-Error Time Series (t, pos-error, vel-error)
	•	Timing & Performance per task + total
	•	Final Comparison Table across datasets & methods

	5.	Copy & paste
Once it finishes, open debug_output.log, copy the console dump and paste it back here so we can verify and iterate.

## Notes

`src/GNSS_IMU_Fusion.py` now detects a low-motion interval in the IMU data to
estimate the initial accelerometer and gyroscope biases. The magnitude of the
static accelerometer vector is used to compute a simple scale factor so that the
measured gravity is close to 9.81 m/s². This improves the attitude
initialisation when the first samples are not perfectly static or the sensor
scale is slightly off.
## Verifying Earth Rotation Rate

Run any dataset with the `--verbose` flag to print the **Earth rotation magnitude** measured from the static gyroscope data.
It should be close to `7.29e-5` rad/s. You can compute the nominal value with:

```bash
python - <<"EOF"
import numpy as np; print(2*np.pi/86400)
EOF
```

If the measured magnitude differs by more than a few percent, the IMU may not be completely still or needs calibration.


## Running all methods

Use `src/run_all_methods.py` to execute the fusion script with the TRIAD,
Davenport and SVD initialisation methods in sequence.  Provide a YAML
configuration file to specify the datasets:

```bash
python src/run_all_methods.py --config your_config.yml
```

Running the script without `--config` processes the bundled example data sets.


## Running all data sets

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

## Running only the TRIAD method

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

## Running a single dataset

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

## Automated figure generation

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

## Summary CSV format

`summarise_runs.py` parses the log files produced by the batch scripts and
writes `results/summary.csv`. Each row contains:

- `method` – attitude initialisation method name
- `imu` – IMU data file
- `gnss` – GNSS data file
- `rmse_pos` – overall position RMSE in metres
- `final_pos` – final position error in metres

`generate_summary.py` reads this CSV to build the PDF report.

## Tests

Run the unit tests with `pytest`. **Installing the required Python packages is
mandatory** before executing any tests. The suite relies on *all* entries in
`requirements.txt` as well as the development dependencies in
`requirements-dev.txt` – including heavier libraries such as `cartopy` that can
take some time to build. Using a dedicated virtual environment or container is
strongly recommended.  The `test` target in the `Makefile` installs both
requirement files for you:

```bash
pip install -r requirements.txt -r requirements-dev.txt
pytest -q
```

You can also simply run `make test` to install both requirement files and
execute the test suite in one command.

## Linting

Static checks run with [Ruff](https://github.com/astral-sh/ruff):

```bash
ruff check src tests
```

Rule `E402` is ignored in `pyproject.toml` as some modules rely on runtime
imports.

## MATLAB Compatibility

Each run now exports a MATLAB `.mat` file alongside the NPZ results. The plotting
and validation functions live in the `MATLAB/` directory. Add it to your path
before calling them:

```matlab
addpath('MATLAB')
plot_results('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat');
validate_3sigma('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat', 'STATE_X001.txt');
```

### Validating with Python

You can also check the exported results directly in Python. Run the helper
script with the `.mat` file and the reference trajectory:

```bash
python src/validate_with_truth.py --est-file results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat --truth-file STATE_X001.txt
```

### Interpolating to Ground Truth

`validate_with_truth.py` first resamples the estimated position, velocity
and quaternion attitude onto the 10&nbsp;Hz timeline provided by
`STATE_X001.txt`. The raw IMU data is recorded at around **400&nbsp;Hz** and
the GNSS receiver typically updates at **1&nbsp;Hz**, so the fusion output is
available at a much higher rate than the reference trajectory.  The script
uses `numpy.interp` for the linear states and a `Slerp` spline for the
quaternions to obtain values exactly at the truth timestamps before computing
the error curves.

The script now produces position, velocity **and** attitude error plots. When
the covariance matrix `P` is available each figure includes the ±3σ envelopes
derived from the corresponding sub-blocks of `P`.
The output directory given via `--output` is created automatically if it does
not exist. After generating the plots the script prints the final position
error and RMSE (and velocity metrics when available) and saves this short
summary to `validation_summary.txt` inside the output directory.

The exported `.npz` and `.mat` structures now expose additional fields alongside
the existing error metrics:

- `attitude_q` – quaternion attitude history for the selected method
- `P_hist` – Kalman covariance matrices over time

### Running the MATLAB pipeline

```matlab
imu_path  = get_data_file('IMU_X001.dat');
gnss_path = get_data_file('GNSS_X001.csv');
TRIAD(imu_path, gnss_path);
```

Scripts such as `TRIAD.m` take the full paths to the IMU and GNSS files as
arguments. The helper function `get_data_file` searches both the repository
root and `MATLAB/data`, letting `TRIAD` and related scripts locate the
bundled sample logs automatically when you pass just the file names.

### MATLAB Scripts

All MATLAB code now lives in the single `MATLAB/` directory. It contains the
pipeline tasks (`Task_1`–`Task_5`) alongside helper scripts such as
`TRIAD.m`, `FINAL.m`, `plot_results.m` and `validate_3sigma.m`. Use these files
for quick experiments or to validate the `.mat` outputs produced by Python.

To run `TRIAD.m` with the new data-file detection logic simply resolve the file
paths with `get_data_file` and pass them to the script:

```matlab
imu  = get_data_file('IMU_X001.dat');
gnss = get_data_file('GNSS_X001.csv');
TRIAD(imu, gnss);
```

`get_data_file` searches `MATLAB/data/` first and falls back to the
repository root, so the command works from any location.

### Sequential Task Execution

To replicate the Python pipeline step by step, call each `Task_1`–`Task_5`
function in order. Always resolve the data paths with `get_data_file` so the
commands work from any folder. Every task writes a `.mat` file that the next one
loads:

```matlab
imu  = get_data_file('IMU_X001.dat');
gnss = get_data_file('GNSS_X001.csv');

Task_1(imu, gnss, 'TRIAD');   % -> results/Task1_init_IMU_X001_GNSS_X001_TRIAD.mat
Task_2(imu, gnss, 'TRIAD');   % -> results/Task2_body_IMU_X001_GNSS_X001_TRIAD.mat
Task_3(imu, gnss, 'TRIAD');   % uses Task1/2 output, writes Task3_results_IMU_X001_GNSS_X001.mat
Task_4(imu, gnss, 'TRIAD');   % uses Task3 results, writes Task4_results_IMU_X001_GNSS_X001.mat
Task_5(imu, gnss, 'TRIAD');   % uses Task4 results, writes Task5_results_IMU_X001_GNSS_X001.mat
```

`Task_3` loads the initial vectors and biases from Tasks 1–2. `Task_4` requires
`Task3_results_<pair>.mat` to compute the NED GNSS trajectory, and `Task_5`
expects `Task4_results_<pair>.mat` to initialise the filter. Running the
commands above reproduces the Python `main` results while letting you inspect
each stage individually.

## Export to MATLAB (.mat) files
To convert the saved Python results into MATLAB `.mat` files, run from the repo root:

```bash
cd /path/to/IMU
pip install numpy scipy
python export_mat.py
```

Or from *any* directory by giving the full path:

```bash
pip install numpy scipy
python /path/to/IMU/export_mat.py
```

## Additional scripts

The repository contains a few helper scripts that are not part of the regular
pipeline but can be handy for quick experiments:

- `fusion_single.py` &ndash; early stand‑alone prototype of the IMU&ndash;GNSS
  fusion routine. It exposes the full bias estimation and Kalman logic in a
  single file and is useful for debugging or exploring tuning options.
- `validate_filter.py` &ndash; command&ndash;line tool to compare a saved filter
  state history against GNSS measurements and plot the residuals.

These utilities are optional and not exercised by the unit tests.

## Next Steps

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


See [docs/CHANGELOG.md](docs/CHANGELOG.md) for a list of recent features.
