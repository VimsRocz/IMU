# MATLAB Pipeline – Task 4 GNSS/IMU Integration

`MATLAB/Task_4.m` integrates the corrected IMU specific force using the attitude from Task 3 and compares the result to the GNSS trajectory.

## Overview

GNSS and IMU logs are loaded, sensor biases are removed and specific force is integrated to obtain position, velocity and acceleration in NED.  The IMU-only trajectory is plotted against the GNSS reference.

```text
Rotation matrix from Task 3
       + IMU data → bias correction
       ↓
Specific force integration
       ↓
GNSS ECEF → NED → comparison plots
```

## Subtasks

### 4.1 Load Rotation Matrices
- Retrieve the body‑to‑NED matrix stored by Task 3.

### 4.3–4.8 Prepare GNSS Data
- Read the GNSS CSV with `readtable` and extract time, ECEF position and velocity.
- Convert the data to the local NED frame using the latitude/longitude from Task 1.
- Differentiate velocity to estimate GNSS acceleration.

### 4.9 Process IMU Data
- Load the IMU file and compute accelerometer and gyroscope biases from an initial static interval.
- Predict gravity and Earth‑rate in the body frame with the rotation matrix and subtract them from the measurements.

### 4.10–4.12 Integrate Specific Forces
- Apply a low‑pass filter and scale gravity to approximately `9.81`.
- Integrate acceleration to yield velocity and position for each method.

### 4.13 Validate and Plot
- Plot GNSS, raw IMU and integrated IMU data in NED, body and ECEF frames.
- Save the PNGs as `results/<tag>_task4_*.png` and list them in `plot_summary.md`.
- Use the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) when naming GNSS, IMU and fused series.
- When the `STATE_X001.txt` reference trajectory is available you can run the Python
  script `src/validate_with_truth.py` or call the MATLAB helper
  `overlay_truth_task4` to overlay the fused output with the ground truth. Before
  running the Python script read the first ECEF row of the state file and supply
  the latitude, longitude and reference point via `--ref-lat`, `--ref-lon` and
  `--ref-r0`. The resulting PNG files `<method>_<frame>_overlay_truth.png` are
  written to the `results/` directory alongside the Kalman filter results.

## Result

Task 4 produces an IMU-only trajectory and diagnostic figures that feed into the Kalman filter of **Task 5**.
