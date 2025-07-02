# Python Pipeline – Task 3 Attitude Determination

Task 3 solves Wahba's problem using the TRIAD algorithm once the reference and body-frame vectors are known.

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
- Form orthogonal triads from the normalised vectors.
- Assemble `R_tri = [t1_NED t2_NED t3_NED] [t1_body t2_body t3_body]^T` and print it.

### 3.3 Quaternion Output
- Convert `R_tri` to a quaternion with `rot_to_quaternion()` and enforce a positive scalar component.
- Log the quaternion and append it to `triad_init_log.txt`.

### 3.4 Check Alignment
- Rotate the body vectors with `R_tri` and compute gravity and Earth‑rate errors in degrees.
- Save the comparison plot as `results/<tag>_task3_errors_comparison.pdf`.

### 3.5 Plot Quaternion Components
- Plot `qw`, `qx`, `qy`, `qz` alongside Davenport and SVD for context.
- The figure is saved as `results/<tag>_task3_quaternions_comparison.pdf`.

## Result

Task 3 produces the initial attitude as a rotation matrix and quaternion.  The values are required for IMU integration in **Task 4** and the Kalman filter in **Task 5**.
