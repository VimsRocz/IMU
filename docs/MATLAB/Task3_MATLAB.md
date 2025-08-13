# MATLAB Pipeline – Task 3 Attitude Determination

Task 3 solves Wahba's problem using the TRIAD algorithm implemented in `MATLAB/Task_3.m`.

## Overview

Gravity and Earth‑rate vectors from Tasks 1 and 2 are normalised and paired to compute a body‑to‑NED rotation matrix and quaternion.  Diagnostic plots compare TRIAD with other methods.

```text
Body vectors (Task 2) + Reference vectors (Task 1)
    ↓
TRIAD → rotation matrix → quaternion
    ↓
Optional comparison plots
```

## Subtasks

### 3.1 Prepare Vector Pairs
- Normalise gravity and Earth‑rate vectors from both frames.
- Optionally recompute the Earth‑rate vector analytically to verify signs.

### 3.2 Compute TRIAD Matrix
- Build orthogonal triads and form `R_tri = [t1_NED t2_NED t3_NED] * [t1_body t2_body t3_body]'`.
- Print the matrix and store it in `results/task3_results.mat`.

### 3.3 Quaternion Output
- Convert `R_tri` to a quaternion with `rot_to_quaternion` and ensure a positive scalar part.
- Append the quaternion to `triad_init_log.txt`.

### 3.4 Check Alignment
- Rotate body vectors with `R_tri` to compute gravity and Earth‑rate errors.
- Save the error plot as `results/<tag>_task3_errors_comparison.png`.
- Apply the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) so the labels match the Python plots.

### 3.5 Plot Quaternion Components
- Plot the quaternion components together with Davenport and SVD for comparison.
- Save the figure as `results/<tag>_task3_quaternions_comparison.png`.

### 3.6 Validate Attitude Determination and Compare Methods
- Rotate the body-frame gravity and Earth‑rate vectors with each method’s rotation matrix.
- Report the angle errors in degrees and warn if the Earth‑rate errors differ by less than `1\u00d710\u22125\u00b0`.
- The MATLAB console output resembles:

```text
Attitude errors using reference vectors (Case 1):
TRIAD      -> Gravity error (deg): 0.000000
TRIAD      -> Earth rate error (deg):  0.000001
Davenport  -> Gravity error (deg): 0.000000
Davenport  -> Earth rate error (deg):  0.000001
SVD        -> Gravity error (deg): 0.000000
SVD        -> Earth rate error (deg):  0.000000

Detailed Earth-Rate Errors:
  TRIAD     : 0.000001°
  Davenport : 0.000001°
  SVD       : 0.000000°

Earth-rate errors by method:
  TRIAD     : 0.000000854°
  Davenport : 0.000000854°
  SVD       : 0.000000000°
  Δ = 8.54e-07° (tolerance = 1.0e-05)
Warning: All Earth-rate errors are very close; differences are within 1.0e-05°
```

## Result

Task 3 produces the initial attitude as a rotation matrix and quaternion ready for the IMU integration step in **Task 4**.
