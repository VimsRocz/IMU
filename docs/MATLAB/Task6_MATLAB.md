# MATLAB Pipeline – Task 6 State Overlay

`MATLAB/Task_6.m` visualises the Kalman filter output from Task 5 against the
raw state trajectory. It mirrors the Python implementation and uses the standard
legend names described in [PlottingChecklist](../PlottingChecklist.md).

## Overview

The script loads `<IMU>_<GNSS>_<METHOD>_kf_output.mat` along with the
corresponding `STATE_X*.txt` file. All IMU, GNSS and truth samples are
interpolated to the estimator time vector before being passed to `plot_overlay`
for the NED, ECEF and body frames.

```text
Task 5 output
      + Truth data → overlay plots
```

## Subtasks

1. **Load Filter Result** – open the saved MAT file and extract the fused
   position, velocity and attitude.
2. **Prepare Truth Data** – convert the ECEF truth trajectory to NED using the
   reference latitude, longitude and origin saved in the result file.
3. **Interpolate** – align IMU, GNSS and truth samples on the filter time grid.
4. **Plot** – call `plot_overlay` for the three frames which stores PDFs named
   `<METHOD>_<FRAME>_overlay_state.pdf` in `results/`. The ``--show-measurements``
   flag mirrors the Python implementation.

## Result

Running `Task_6` produces three comparison figures showing how well the fused
trajectory follows the reference solution.
