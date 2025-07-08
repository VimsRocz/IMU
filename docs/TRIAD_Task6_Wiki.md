# TRIAD Method – Task 6 Truth Overlay

This short task augments the figures from **Task 5** with the recorded ground
truth trajectory. It does not modify the filter output; only additional lines are
plotted for comparison.

## Overview

The stored Kalman filter result is loaded and aligned with the corresponding
GNSS/IMU logs and the `STATE_X001.txt` reference. The existing Task 5 plots are
recreated with the truth trajectory shown in black.  Because NED uses a
positive *Down* axis, the script flips that component so altitude appears
upward on screen.

```text
Task 5 fused trajectory
       + Truth data → overlay figures
```

## Subtasks

1. Load the Task 5 output file (`*_kf_output.mat` or `.npz`).
2. Interpolate IMU, GNSS and truth samples onto the same time vector.
3. Save NED, ECEF and body-frame overlay plots using the standard layout.
4. Create additional figures overlaying the raw `STATE_X*.txt` data without
   alignment. These highlight how the estimator drifts relative to the logged
   truth.

## Result

Task 6 provides visual confirmation of the filter accuracy by directly comparing
it against the reference trajectory.
