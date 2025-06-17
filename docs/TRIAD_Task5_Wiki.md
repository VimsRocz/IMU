# TRIAD Method – Task 5 Kalman Filter Fusion

This page documents **Task 5** of the `GNSS_IMU_Fusion.py` pipeline. After Task 4 provides an IMU-only trajectory, Task 5 fuses the inertial and GNSS data with a simple Kalman filter.

## Overview

Task 5 loads the corrected IMU data and the GNSS measurements, performs prediction with the integrated accelerations and applies measurement updates at each GNSS epoch. The filter outputs fused position, velocity and attitude estimates along with innovation diagnostics.

```
IMU integration (Task 4)
       ↓
Kalman filter predict
       + GNSS updates → update
       ↓
Fused trajectory + innovations
```

## Subtasks

### 5.1 Configure Logging
- Initialise the logging system and create any required files.

### 5.2–5.4 Data Preparation
- Convert GNSS data to NED and compute acceleration as in Task 4.
- Correct the IMU specific force using the biases and rotation matrices.
- Integrate the IMU data to obtain position and velocity priors.

### 5.6 Run Kalman Filter
- Predict state and covariance using the integrated IMU data.
- Update with interpolated GNSS position and velocity at each epoch.
- Apply zero‑velocity updates when a static interval is detected.

### 5.8 Plot and Summarise Results
- Save plots of fused position, velocity and acceleration for each method.
- Record innovations, residuals and attitude angles in the results folder.
- Write a small markdown summary listing the generated PDFs.

## Result

The Kalman filter outputs a fused trajectory and diagnostic statistics for each initialisation method. These figures complete the processing chain started in Task 1.
