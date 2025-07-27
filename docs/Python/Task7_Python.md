# Python Pipeline – Task 7 Filter Evaluation

Task 7 analyses the filter residuals and attitude history. The Python scripts load the residual arrays produced in Task 5 and create diagnostic figures directly in ``results/`` using the dataset tag ``<TAG>`` as part of each filename.

## Overview

Residual position and velocity are compared with the GNSS data. When a truth trajectory is available the difference ``truth - fused`` is also plotted.

## Subtasks

### 7.1 Load Data
- Read ``residual_pos`` and ``residual_vel`` along with ``attitude_q`` from ``*_kf_output.npz``.
- Check the array lengths and truncate mismatched samples.

### 7.2 Compute Residuals
- Interpolate GNSS position and velocity to the estimator time vector.
- Print mean and standard deviation of the residuals.

### 7.3 Plot Residuals
- Save `<tag>_task7_3_residuals_position_velocity.pdf` and an error norm plot in the same folder.
- `<tag>` now includes the IMU dataset, GNSS log and method, e.g. `IMU_X002_GNSS_X002_Davenport`.

### 7.4 Plot Attitude Angles
- Convert the quaternion history to Euler angles.
- Write `<tag>_task7_4_attitude_angles_euler.pdf` with roll, pitch and yaw over time.

### 7.5 Truth – Fused Difference
- If the reference trajectory is provided, plot component-wise differences.
- Figures are saved as `<tag>_task7_5_diff_truth_fused_over_time.pdf` (NED frame).

## Result

Task 7 produces residual and attitude plots that summarise the filter performance for each data set.
