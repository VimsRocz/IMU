# Python Pipeline – Task 4 GNSS/IMU Integration

With the attitude from Task 3 known, Task 4 integrates the IMU specific force and contrasts the result with the GNSS trajectory.

## Overview

GNSS and IMU data are loaded, biases are corrected and the specific force is integrated to obtain position, velocity and acceleration in the NED frame.  Several plots compare the IMU‑only solution against the GNSS reference.

```text
Rotation matrix from Task 3
       + IMU data → bias correction
       ↓
Specific force integration
       ↓
GNSS ECEF → NED → comparison plots
```

## Subtasks

### 4.1 Retrieve Rotation Matrices
- Load the body‑to‑NED matrix from Task 3 for the selected method.

### 4.3–4.8 Prepare GNSS Data
- Read the GNSS CSV and extract time, ECEF position and velocity.
- Convert ECEF to NED using the latitude and longitude from Task 1.
- Estimate GNSS acceleration by differentiating velocity.

### 4.9 Process IMU Data
- Load the IMU file and compute accelerometer and gyroscope biases from an initial static interval.
- Predict the gravity and Earth‑rate vectors in the body frame using the rotation matrix and subtract them from the measurements.

### 4.10–4.12 Integrate Specific Forces
- Low‑pass filter the corrected accelerations and scale them so gravity magnitude is close to `9.81 m/s²`.
- Integrate acceleration to obtain velocity and position traces.

### 4.13 Validate and Plot
- Create comparison plots in NED, body and ECEF frames.
- Save the PDFs to `results/<tag>_task4_*.pdf` and record them in `plot_summary.md`.
- Consult the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) when labelling GNSS, IMU and fused traces.

## Result

Task 4 yields an IMU-only trajectory and diagnostic figures that feed into the Kalman filter of **Task 5**.
