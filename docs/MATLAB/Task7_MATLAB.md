# MATLAB Pipeline – Task 7 Filter Evaluation

`evaluate_filter_results.m` mirrors the Python implementation and processes the residuals stored in `*_kf_output.npz`.

## Overview

The script loads residual and attitude data, computes basic statistics and writes figures under ``results/task7/<TAG>/``.

## Subtasks

### 7.1 Load Data
- Use `numpy.load` to read `residual_pos`, `residual_vel` and `attitude_q` arrays.
- Truncate arrays to the same length and report the number of samples.

### 7.2 Compute Residuals
- Reconstruct the truth trajectory by subtracting residuals from the fused solution when available.
- Output mean and standard deviation for position and velocity errors.

### 7.3 Save Residual Plots
- Store `<tag>_task7_3_residuals_position_velocity.pdf` and `<tag>_task7_3_error_norms.pdf`.

### 7.4 Plot Attitude Angles
- Convert quaternions to Euler angles with `quat2eul`.
- Save `<tag>_task7_4_attitude_angles_euler.pdf`.

### 7.5 Truth – Fused Difference
- When both trajectories exist, plot their difference over time.
- Write `<tag>_task7_5_diff_truth_fused_over_time.pdf`.

## Result

Running `evaluate_filter_results` summarises residual statistics and attitude behaviour for each dataset. `<tag>` now combines the IMU dataset, GNSS log and initialisation method, for example `IMU_X002_GNSS_X002_Davenport`.
