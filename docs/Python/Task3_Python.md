# Python Pipeline – Task 3 Attitude Determination

Task 3 solves Wahba's problem using a small SVD-based routine that refines the traditional TRIAD approach.

## Overview

The normalised gravity and Earth‑rate vectors from Tasks 1 and 2 are paired to compute a body‑to‑NED rotation matrix and a quaternion.  Diagnostic plots compare TRIAD with the alternative methods.

```text
Body vectors (Task 2) + Reference vectors (Task 1)
    ↓
TRIAD → rotation matrix → quaternion
    ↓
Optional comparison plots
```

## Subtasks

### 3.1 Prepare Vector Pairs
- Normalise the gravity and Earth‑rate vectors in both frames.
- Optionally recompute the Earth‑rate vector in closed form to verify the sign convention.

### 3.2 Compute TRIAD Matrix
The vectors are fed to `triad_svd()`, which forms the Wahba matrix and
solves for the optimal rotation with a singular value decomposition.
This is numerically more robust than the classic cross-product formula.

### 3.3 Quaternion Output
- Convert `R_tri` to a quaternion with `rot_to_quaternion()` and enforce a positive scalar component.
- Log the quaternion and append it to `triad_init_log.txt`.

### 3.4 Check Alignment
- Rotate the body vectors with `R_tri` and compute gravity and Earth‑rate errors in degrees.
- Save the comparison plot as `results/<tag>_task3_errors_comparison.pdf`.
- Use the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) so the curves match the rest of the documentation.

### 3.5 Plot Quaternion Components
- Plot `qw`, `qx`, `qy`, `qz` alongside Davenport and SVD for context.
- The figure is saved as `results/<tag>_task3_quaternions_comparison.pdf`.

## Result

Task 3 produces the initial attitude as a rotation matrix and quaternion.  The values are required for IMU integration in **Task 4** and the Kalman filter in **Task 5**.
