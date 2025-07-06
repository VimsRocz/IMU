# Python Pipeline – Task 6 Truth Overlay

Task 6 visualises the Kalman filter output from Task 5 together with the known
reference trajectory. The figures are identical to the Task 5 plots but include
a black **Truth** line for direct comparison.

## Overview

The script loads the stored filter results (`*_kf_output.mat` or `.npz`),
retrieves the corresponding GNSS and IMU logs and aligns them with the ground
truth file. The helper :func:`plot_overlay` then creates 3×3 figures in the NED,
ECEF and body frames.

```text
Task 5 output
       + Ground truth → overlay plots
```

## Subtasks

1. **Load Estimator Result** – parse the Task 5 output file and obtain fused
   position, velocity and acceleration.
2. **Assemble Frames** – use :func:`assemble_frames` to align IMU, GNSS and truth
   data in NED, ECEF and body frames.
3. **Generate Plots** – call :func:`plot_overlay` for each frame to save PDFs
   named `<METHOD>_<FRAME>_overlay_truth.pdf` in the results directory.

## Result

Task 6 yields comparison plots that clearly show how well the fused trajectory
matches the reference solution.
