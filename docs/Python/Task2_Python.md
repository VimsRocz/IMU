# Python Pipeline – Task 2 Body Vectors

Task 2 measures gravity and the Earth rotation rate in the IMU frame.  The code lives in `src/GNSS_IMU_Fusion.py` and follows these steps.

## Overview

The IMU data is loaded, a static interval is detected to estimate sensor biases and the body‑frame vectors are computed.  Optionally a magnetometer can provide a third vector.

```text
IMU data → low-pass filter → static interval
     ↓
Accelerometer & gyroscope means
     ↓
Body-frame gravity g_body and Earth rate ω_ie_body
```

## Subtasks

### 2.1 Load IMU Data
- Read the `.dat` file with `numpy.loadtxt` and extract the time, angular increments and velocity increments.
- Derive the sampling period from the timestamps or assume `400 Hz` if none are present.
- Print the number of samples and the derived sampling period.

### 2.2 Detect Static Interval
- Convert increments to rates using the sampling period.
- Apply a Butterworth low‑pass filter around `5 Hz` to suppress high‑frequency noise.
- Use `detect_static_interval` to find a low‑motion window and compute mean accelerometer and gyroscope vectors.
- Log the interval indices, means and variances to `triad_init_log.txt`.
- Optionally scale the accelerometer vector so its magnitude is exactly `9.81 m/s²`.
- Plot the detected interval with `plot_zupt_and_variance()` and save the PDF to `results/<tag>_ZUPT_variance.pdf`.

### 2.3 Derive Body‑Frame Vectors
- Negate the accelerometer mean to obtain `g_body`.
- Use the gyroscope mean as `ω_ie_body`.
- Extract a static magnetometer vector if available for use in TRIAD.

### 2.4 Validate Measurements
- Check vector shapes and magnitudes and print them to the console.
- Warn if gravity differs from `9.81 m/s²` or rotation from `7.292e‑5 rad/s` by more than a few percent.

## Result

Task 2 outputs gravity and Earth‑rate vectors in the sensor frame.  Together with the reference vectors from Task 1 they form the input to the TRIAD attitude solution in **Task 3**.
