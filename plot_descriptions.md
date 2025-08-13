# Plot Descriptions

All reported results and plots are for the TRIAD initialization method.

## 1. Initial Location Map
**Filename:** <tag>_task1_location_map.png
**Description:** Map showing the start position derived from the GNSS file. This figure is produced in Task&nbsp;1.

## 2. ZUPT Detection and Accelerometer Variance
**Filename:** IMU_Xnnn_ZUPT_variance.png
**Description:** Shows the detected zero-velocity intervals and the variance of the accelerometer signal over time. Orange regions indicate ZUPT.
## 3. Attitude Initialisation Error Comparison
**Filename:** <tag>_task3_errors_comparison.png
**Description:** Gravity and Earth-rate alignment errors for the TRIAD, Davenport and SVD solutions.

## 4. Quaternion Components Comparison
**Filename:** <tag>_task3_quaternions_comparison.png
**Description:** Comparison of `qw`, `qx`, `qy`, `qz` across methods after Task&nbsp;3.

## 5. Attitude Angles (Roll/Pitch/Yaw) vs. Time
**Filename:** IMU_Xnnn_EulerAngles_time.png
**Description:** Roll, pitch, and yaw estimated from the TRIAD method, visualized for the entire trajectory.

## 6. Position Residuals (Filter - GNSS) vs. Time
**Filename:** IMU_Xnnn_GNSS_Xnnn_pos_residuals.png
**Description:** Shows how closely the Kalman filter position matches GNSS; helps visualize drift or filter corrections.

## 7. Velocity Residuals (Filter - GNSS) vs. Time
**Filename:** IMU_Xnnn_GNSS_Xnnn_vel_residuals.png
**Description:** Shows difference in velocity estimates between filter and GNSS, indicating filter tracking accuracy.

## 8. Attitude Angles Over Time
**Filename:** IMU_Xnnn_GNSS_Xnnn_attitude_time.png
**Description:** Roll, pitch, and yaw estimates over the full dataset, useful for spotting jumps or drift.

