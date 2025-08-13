# TRIAD Method – Task 4 GNSS/IMU Integration

This page documents **Task 4** of the `src/GNSS_IMU_Fusion.py` pipeline. With the attitude known from Task 3, the script integrates the IMU measurements and contrasts the result with the GNSS trajectory.

## Overview

Task 4 loads the GNSS and IMU data, corrects sensor biases using the rotation matrices from Task 3 and integrates the specific force to obtain position, velocity and acceleration in the NED frame. Several figures compare the IMU-only solution against the GNSS reference.

```
Rotation matrix from Task 3
       + IMU data → bias correction
       ↓
Specific force integration
       ↓
GNSS ECEF → NED → comparison plots
```

## Subtasks

### 4.1 Access Rotation Matrices from Task 3
- Retrieve the body-to-NED rotation matrix computed for each method.

### 4.3–4.8 Load and Prepare GNSS Data
- Read the GNSS CSV file and extract time, ECEF position and velocity columns.
- Convert the ECEF data to the local NED frame using the reference latitude/longitude from Task 1.
- Estimate GNSS acceleration by differentiating the velocity.

### 4.9 Load IMU Data and Correct Bias
- Read the IMU `.dat` file and compute accelerometer and gyroscope biases from an initial static interval.
- Use the rotation matrix to predict the expected gravity and Earth rate in the body frame and remove them from the measurements.

### 4.10–4.12 Integrate Specific Forces
- Apply a low-pass filter and scale so gravity magnitude is close to 9.81 m/s².
- Integrate the corrected accelerations to obtain velocity and position for each method.

### 4.13 Validate and Plot
- Create comparison plots in NED, body and ECEF frames showing GNSS, raw IMU and integrated IMU data.
- Save the PNGs to `results/<tag>_task4_*.png` and list them in `plot_summary.md`.
- Consult the [standardized legend terms](PlottingChecklist.md#standardized-legend-terms) when naming GNSS, IMU and fused traces.

## Result

Task 4 produces an IMU-only trajectory and diagnostic figures comparing it to the GNSS measurements. These outputs are used as the prediction step for the Kalman filter in **Task 5**.
