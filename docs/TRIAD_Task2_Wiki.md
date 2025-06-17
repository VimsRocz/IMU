# TRIAD Method – Task 2 Measuring Body Vectors

This page documents **Task 2** of the `GNSS_IMU_Fusion.py` pipeline. After defining the reference vectors in Task 1, the script measures the same physical vectors in the IMU body frame. These body-frame vectors are required for the TRIAD attitude initialisation.

## Overview

Task 2 loads the IMU data, detects a static interval to estimate sensor biases and then expresses gravity and the Earth rotation rate in the body frame. Optional magnetometer data can also be processed if provided.

```
IMU data → low-pass filter → static interval
↓
Accelerometer & gyroscope means
↓
Body-frame gravity g_body and Earth rate ω_ie_body
```

During this task the script also performs sanity checks, writes detailed logs to
`triad_init_log.txt` and prints the resulting vectors to the console. The goal is
to obtain a clean estimate of gravity and the Earth rotation rate in sensor
axes before continuing with the TRIAD attitude initialisation.

## Subtasks

### 2.1 Load and Parse IMU Data
- Read the IMU `.dat` file with `numpy.loadtxt`.
- Extract velocity increments (`columns 5–7`) and angular increments (`columns 2–4`).
- Estimate the sampling period from the time column if present, otherwise assume `400 Hz`.
- Print the number of samples and the derived sampling period so potential data
  issues can be spotted immediately.

### 2.2 Estimate Static Body‑Frame Vectors
- Convert increments to rates using the sampling period.
- Apply a Butterworth low‑pass filter (cut-off around `5 Hz`) to suppress noise.
- Detect a low‑motion interval with `detect_static_interval` and compute the mean
  accelerometer and gyroscope vectors within this window.
- Log the interval indices, mean values and variances; store them in
  `triad_init_log.txt` for later inspection. This helps verify the static
  assumption and provides a traceable record of the calibration step.
- Optionally apply a scale factor so the gravity magnitude is close to `9.81 m/s²`.
- Print the means and magnitudes of the accelerometer and gyroscope vectors so
  the operator can confirm that the IMU was indeed stationary.
- Save a diagnostic plot of the detected static window using
  `plot_zupt_and_variance()`. The file is stored as
  `results/<tag>_ZUPT_variance.pdf` and summarised in `plot_summary.md`.

### 2.3 Define Gravity and Earth Rate in Body Frame
- The measured accelerometer mean is negated to give `g_body`.
- The mean gyroscope vector is used as `ω_ie_body`.
- If magnetometer data is supplied, a static vector is also extracted for later
  use. This magnetometer vector will form a third input to the TRIAD algorithm
  if available.

### 2.4 Validate and Print Body‑Frame Vectors
- Check that the vectors have the correct shape and reasonable magnitudes.
- Print the gravity and Earth‑rate vectors along with their norms.
- Warn if the gravity or rotation magnitudes deviate significantly from the
  expected `9.81 m/s²` and `7.292e‑5 rad/s` respectively.
- These vectors correspond to the reference vectors from Task 1 but expressed in
  sensor axes. They are used together with the reference vectors when solving the
  initial attitude in Task 3.

## Result

After completing Task 2 the script has estimates for gravity and the Earth rotation rate in the body frame. These measured vectors, together with the reference vectors from Task 1, form the input to the TRIAD algorithm in Task 3.
