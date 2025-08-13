# MATLAB Pipeline – Task 2 Body Vectors

Task 2 estimates gravity and the Earth rotation rate in the sensor axes using the routine `MATLAB/Task_2.m`.

## Overview

The IMU log is processed to detect a static interval.  Sensor biases are computed and the body‑frame vectors are derived.  A magnetometer reading can be included if available.

```text
IMU data → low-pass filter → static interval
     ↓
Accelerometer & gyroscope means
     ↓
Body-frame gravity g_body and Earth rate ω_ie_body
```

## Subtasks

### 2.1 Load IMU Data
- Use `load` or `dlmread` to read the `.dat` file containing time, angular increments and velocity increments.
- Determine the sampling period from the time column or assume `1/400` seconds.
- Print the sample count and sampling period for verification.

### 2.2 Detect Static Interval
- Convert increments to rates based on the sampling period.
- Apply a Butterworth low‑pass filter (cut-off around `5 Hz`).
- Call `detect_static_interval` to find a quiet window and compute mean accelerometer and gyroscope vectors.
- Save the indices and statistics to `results/triad_init_log.txt`.
- Optionally scale **only** the mean accelerometer vector so its magnitude equals `9.81`.
- The rest of the IMU data is left unscaled to preserve dynamic measurements.
- Plot the detected interval with `plot_zupt_and_variance` and save the PNG.
- When labelling the plot refer to the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms).
- After detection, the code prints the duration of the static window and
  compares it to the total dataset length. If more than ~90 % of the data is
  flagged as static, a warning suggests verifying the motion log or relaxing the
  detection thresholds.

### 2.3 Derive Body‑Frame Vectors
- Negate the accelerometer mean to produce `g_body`.
- Use the gyroscope mean as `ω_ie_body`.
- Extract a static magnetometer vector if one was recorded.

### 2.4 Validate Measurements
- Check the vector dimensions and magnitudes.
- Print the values and warn if they deviate significantly from expected norms.

## Result

Task 2 outputs gravity and Earth‑rate vectors in the body frame for use by TRIAD in **Task 3**.
