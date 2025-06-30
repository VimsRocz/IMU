# TRIAD Method – Task 3 Attitude Determination

This page documents **Task 3** of the `GNSS_IMU_Fusion.py` pipeline. After Tasks 1 and 2 provide matching reference and body-frame vectors, Task 3 solves Wahba’s problem to obtain the initial attitude. Only the TRIAD solution is described here.

## Overview

Task 3 pairs the gravity and Earth‑rotation vectors from the previous tasks and feeds them to the TRIAD algorithm. The result is a body‑to‑NED rotation matrix and a quaternion. Diagnostic figures compare the TRIAD solution with Davenport and SVD, but the focus is on the TRIAD output.

```
Body vectors (Task 2) + Reference vectors (Task 1)
↓
TRIAD → rotation matrix → quaternion
↓
Optional comparison plots
```

## Subtasks

### 3.1 Prepare Vector Pairs
- Normalise gravity and Earth‑rate vectors in both frames.
- Optionally recompute the Earth rate using the closed‑form equation to check sign conventions.

### 3.2 Compute the TRIAD Rotation Matrix
- Form orthogonal triads from the normalised vectors:
  - `t1_body = v1_B`
  - `t2_body = unit(v1_B × v2_B)`
  - `t3_body = t1_body × t2_body`
  - Equivalent axes `t1_NED`, `t2_NED`, `t3_NED` in the reference frame.
- Compose the rotation matrix
  `R_tri = [t1_NED t2_NED t3_NED] [t1_body t2_body t3_body]^T`.
- Print the matrix and append it to `triad_init_log.txt`.

### 3.3 Convert to Quaternion
- Convert `R_tri` using `rot_to_quaternion()` and enforce a positive scalar part.
- Log and print the quaternion components.

### 3.4 Validate Gravity and Earth‑Rate Alignment
- Rotate the body vectors with `R_tri` and compute the angles to the references.
- Report the gravity and Earth‑rate errors in degrees.
- These errors appear in `results/<tag>_task3_errors_comparison.pdf`.

### 3.5 Plot Quaternion Components
- Plot `qw`, `qx`, `qy`, `qz` alongside the other methods for context.
- The figure `results/<tag>_task3_quaternions_comparison.pdf` is created.

## Result

Task 3 outputs the rotation matrix and quaternion describing the initial attitude. The error and quaternion plots are saved to the `results/` folder and listed in `plot_summary.md`. Use `run_triad_only.py` (or the MATLAB script `matlab/run_triad_only.m`) to process all datasets and generate these figures. The attitude estimate serves as the starting point for the IMU/GNSS comparison in **Task 4** and the subsequent Kalman filter fusion in **Task 5**.
