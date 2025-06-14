# IMU

IMU data processing and initialization tools in Python for experimenting with attitude initialization and GNSS/IMU fusion.

For a step-by-step explanation of the demonstration workflow see **TASKS.md**.

## Installation

Install dependencies with:

```bash
pip install -r requirements.txt
Three directories (`triad_workflow`, `davenport_workflow`, `svd_workflow`) each
contain a self-contained `workflow.py` script. These replicate the full
processing pipeline but solve Wahba's problem using only one algorithm
(TRIAD, Davenport or SVD respectively). Run the script inside the chosen
folder to execute the complete workflow and generate the diagnostic PDFs.

## Expected datasets

Several datasets are included to test different scenarios:

- **IMU_X001.dat** – Clean IMU measurements with minimal sensor noise.
- **IMU_X002.dat** – The same trajectory with additional noise injected in the inertial sensors.
- **IMU_X003.dat** – IMU readings with constant bias errors.
- **GNSS_X001.csv** – Nominal GNSS positions and velocities in ECEF coordinates.
- **GNSS_X002.csv** – GNSS data with simulated measurement noise.

## Scripts

### fusion_single.py
A simplified demonstration of the Kalman filter using a single IMU and GNSS pair.

### GNSS_IMU_Fusion.py
The main entry point for running the fusion on any of the provided datasets.

### full_project_workflow.py
A consolidated script that reproduces all of the step-by-step tasks from the
project. It computes reference vectors, solves Wahba's problem using multiple
methods, compares IMU and GNSS data and runs a basic Kalman filter. Run the
script without arguments to process the example datasets and generate the PDF
figures described below.

## Utility functions

The `imu_fusion.data` module provides `estimate_acc_bias` for computing a
robust accelerometer bias from the first seconds of data. It applies a
simple low-pass filter and returns the median vector, helping to
initialize filters when raw measurements contain spikes.


## Running `GNSS_IMU_Fusion.py`

`GNSS_IMU_Fusion.py` is the canonical entry point for the fusion demo. Any
functionality from the older `fusion_single.py` script has been merged here.

The fusion script accepts the IMU and GNSS file paths via command-line options:

```bash
python GNSS_IMU_Fusion.py --gnss-file GNSS_X001.csv --imu-file IMU_X001.dat
python GNSS_IMU_Fusion.py --gnss-file GNSS_X002.csv --imu-file IMU_X002.dat
python GNSS_IMU_Fusion.py --gnss-file GNSS_X001.csv --imu-file IMU_X003.dat
```

Specify `--init-method` to choose the attitude initialization algorithm.
The `MEAN` option averages TRIAD, Davenport and SVD results.
GNSS inputs can be smoothed with `--smooth-window` and IMU bias
estimated using `--bias-samples`:

```bash
python GNSS_IMU_Fusion.py --init-method MEAN --smooth-window 7 --bias-samples 500
```

Choose any combination of the GNSS and IMU datasets to evaluate the effect of noise and bias on the fusion process.

## Generated outputs

After execution, several PDF plots summarizing the results are written to the working directory. A `plot_summary.md` file is also generated that lists each PDF with a short description. These files provide visual feedback of the Kalman filter results, residuals and attitude estimates.

## Logging

All scripts now write detailed logs to the `logs/` directory. Console output
is mirrored to `logs/run.log` and a short summary of each run is appended to
`logs/run_summary.txt`. Review these files to verify the measured biases,
attitude errors and final Kalman drift after each execution.

