# IMU

IMU data processing and initialization tools in Python for experimenting with attitude initialization and GNSS/IMU fusion.

For a step-by-step explanation of the demonstration workflow see **TASKS.md**.

## Installation

Install dependencies with:

```bash
pip install -r requirements.txt
```

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

## Running `GNSS_IMU_Fusion.py`

The fusion script accepts the IMU and GNSS file paths via command-line options:

```bash
python GNSS_IMU_Fusion.py --gnss-file GNSS_X001.csv --imu-file IMU_X001.dat
python GNSS_IMU_Fusion.py --gnss-file GNSS_X002.csv --imu-file IMU_X002.dat
python GNSS_IMU_Fusion.py --gnss-file GNSS_X001.csv --imu-file IMU_X003.dat
```

Choose any combination of the GNSS and IMU datasets to evaluate the effect of noise and bias on the fusion process.

## Generated outputs

After execution, several PDF plots summarizing the results are written to the working directory. A `plot_summary.md` file is also generated that lists each PDF with a short description. These files provide visual feedback of the Kalman filter results, residuals and attitude estimates.

