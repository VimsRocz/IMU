# IMU_GNSS_Fusion
IMU data processing and initialization tools (Python)

### Table of Contents
- [Installation](#installation)
- [Datasets](#datasets)
- [Running the pipeline](#running-the-pipeline)
  - [run_all_datasets.py](#run_all_datasetspy)
  - [run_triad_only.py](#run_triad_onlypy)
- [Per-task overview](#per-task-overview)
- [Additional scripts](#additional-scripts)
- [Tests](#tests)
- [Linting](#linting)
- [Advanced topics](docs/AdvancedTopics.md)

### Installation
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

### Running the pipeline

#### Validation
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

### Per-task overview
1. **Task 1 – Reference vectors** – derive latitude/longitude from GNSS and define gravity and Earth rotation vectors.
2. **Task 2 – Body vectors** – load IMU data, detect a static interval and express the reference vectors in the body frame.
3. **Task 3 – Attitude determination** – compute the initial orientation using the TRIAD algorithm.
4. **Task 4 – GNSS/IMU integration** – integrate IMU measurements and compare with GNSS in the NED frame.
5. **Task 5 – Kalman filter fusion** – combine IMU and GNSS data with a Kalman filter to produce the final trajectory.

### Additional scripts

The repository contains a few helper scripts that are not part of the regular
pipeline but can be handy for quick experiments:

- `fusion_single.py` &ndash; early stand‑alone prototype of the IMU&ndash;GNSS
  fusion routine. It exposes the full bias estimation and Kalman logic in a
  single file and is useful for debugging or exploring tuning options.
- `validate_filter.py` &ndash; command&ndash;line tool to compare a saved filter
  state history against GNSS measurements and plot the residuals.

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

For GitHub Codespaces instructions and MATLAB usage see [docs/AdvancedTopics.md](docs/AdvancedTopics.md).

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

See [docs/CHANGELOG.md](docs/CHANGELOG.md) for a list of recent features.
