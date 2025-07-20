# Python Pipeline – Task 6 State Overlay

Task 6 visualises the Kalman filter output from Task 5 together with the raw
`STATE_X` trajectory. Only the ``*_overlay_state.pdf`` figures are produced.

## Overview

The script loads the stored filter results (`*_kf_output.mat` or `.npz`),
retrieves the corresponding GNSS and IMU logs and aligns them with the ground
truth file. The helper :func:`plot_overlay` then creates 3×3 figures in the NED,
ECEF and body frames.  Since NED uses a positive *Down* axis, the script flips
that component before plotting so altitude increases upward in the figures.

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
   named `<TAG>_task6_<FRAME>_overlay_state.pdf` in the results directory.  The
   ``--show-measurements`` flag adds the raw IMU and GNSS curves.

## Result

Task 6 yields comparison plots that clearly show how well the fused trajectory
matches the reference solution.
