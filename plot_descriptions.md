# Plot Descriptions

All reported results and plots are for the TRIAD initialization method.

## 1. ZUPT Detection and Accelerometer Variance
**Filename:** IMU_Xnnn_ZUPT_variance.pdf  
**Description:** Shows the detected zero-velocity intervals and the variance of the accelerometer signal over time. Orange regions indicate ZUPT.

## 2. Attitude Angles (Roll/Pitch/Yaw) vs. Time
**Filename:** IMU_Xnnn_EulerAngles_time.pdf  
**Description:** Roll, pitch, and yaw estimated from the TRIAD method, visualized for the entire trajectory.

## 3. Position Residuals (Filter - GNSS) vs. Time
**Filename:** IMU_Xnnn_GNSS_Xnnn_pos_residuals.pdf  
**Description:** Shows how closely the Kalman filter position matches GNSS; helps visualize drift or filter corrections.

## 4. Velocity Residuals (Filter - GNSS) vs. Time
**Filename:** IMU_Xnnn_GNSS_Xnnn_vel_residuals.pdf  
**Description:** Shows difference in velocity estimates between filter and GNSS, indicating filter tracking accuracy.

## 5. Attitude Angles Over Time
**Filename:** IMU_Xnnn_GNSS_Xnnn_attitude_time.pdf  
**Description:** Roll, pitch, and yaw estimates over the full dataset, useful for spotting jumps or drift.

