# IMU-
IMU data processing and initialization tools (Python)

## Installation

`run_all_datasets.py` installs the required packages automatically the first
time you run it. If you'd rather set them up beforehand, install the
dependencies manually with:

```bash
pip install -r requirements.txt
```

### Installing FilterPy on Ubuntu

If installation of `filterpy` fails (for example due to missing build tools) run:

```bash
sudo apt update && sudo apt install python3-pip
pip3 install filterpy
```

If the build error complains about Cython install it explicitly first:

```bash
pip3 install cython
pip3 install filterpy
```

## 🚀 Developing & Debugging in GitHub Codespaces

1. **Open in Codespace**  
   Click **Code → Open with Codespaces** on the repo.

2. **Install dependencies**  
   Codespaces will automatically build the container (per `.devcontainer/`), install Python & your `requirements.txt`.

3. **Enable verbose diagnostics**  
   We’ve added a `--verbose` flag to `run_all_datasets.py` that enables all the extra tables and timing you requested.

4. **Run with diagnostics**  
   In the integrated terminal:
   ```bash
   python run_all_datasets.py --verbose 2>&1 | tee debug_output.log
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

`GNSS_IMU_Fusion.py` now detects a low-motion interval in the IMU data to
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

Use `run_all_methods.py` to execute the fusion script with TRIAD, Davenport and SVD sequentially:

```bash
python run_all_methods.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv
```


## Running all data sets

To process every IMU/GNSS pair defined in `run_all_datasets.py`, simply run:

```bash
python run_all_datasets.py
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

If you want to process all datasets using just the TRIAD initialisation method, run the helper script `run_triad_only.py`:

```bash
python run_triad_only.py
```
This is equivalent to running `run_all_datasets.py --method TRIAD`.


After all runs complete you can compare the datasets side by side:

```bash
python plot_compare_all.py
```
This creates one `all_datasets_<method>_comparison.pdf` per method in
`results/`.

## Automated figure generation

The module `auto_plots.py` contains helper routines for creating the standard
set of plots and a summary table of metrics for each batch run.  Integrate
these helpers into your own batch script to automatically export the six
"must‑have" figures (tasks 3–5 and validation plots) and a CSV/LaTeX table of
RMSE and innovation statistics.

Interactive exploration lives in the `notebooks/` folder. Open
`notebooks/demo.ipynb` to try the plotting utilities in Jupyter.

## Tests

Run the unit tests with `pytest`:

```bash
pytest -q
```

## Next Steps

- **Logging:** Extend the built-in `logging` with the `rich` console handler to
  get colourful status messages during long runs.
- **Documentation:** The devcontainer includes Sphinx and MkDocs. Generate the
  API docs with `sphinx-build` and publish user guides with `mkdocs`.
- **CI:** Set up a simple GitHub Actions workflow that installs
  `requirements.txt`, runs `flake8` and the `pytest` suite on every pull
  request.

