# MATLAB Pipeline – Task 5 Kalman Filter Fusion

`MATLAB/Task_5.m` fuses the IMU-only trajectory from Task 4 with GNSS measurements using a simple Kalman filter.

## Overview

The routine loads the corrected IMU data and GNSS updates, runs a predict/update loop and outputs fused position, velocity and attitude along with innovation diagnostics.

```text
IMU integration (Task 4)
       ↓
Kalman filter predict
       + GNSS updates → update
       ↓
Fused trajectory + innovations
```

## Subtasks

### 5.1 Configure Logging
- Set up any log files or structures used for diagnostics.

### 5.2–5.4 Prepare Inputs
- Convert GNSS data to NED and compute acceleration as done in Task 4.
- Correct the IMU specific force with the estimated biases and rotation matrix.
- Integrate IMU data to obtain position and velocity priors.

### 5.6 Run the Kalman Filter
- Predict state and covariance using the priors.
- Update the state with interpolated GNSS positions and velocities.
- Apply zero‑velocity updates during detected static periods.

### 5.8 Plot and Summarise
- Save plots of fused position, velocity and acceleration.
- Record innovations, residuals and attitude angles in the results folder.
- Write a short markdown summary of the generated PDFs.
- Follow the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) to keep MATLAB and Python figures consistent.

## Result

Task 5 outputs the fused trajectory and diagnostic statistics, concluding the pipeline that began with Task 1.
