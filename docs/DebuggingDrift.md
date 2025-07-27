# Debugging Large Position Drift in the MATLAB Pipeline

This guide explains how to track down extremely large position errors (tens of kilometres) when running the `MATLAB` pipeline. The Python implementation normally finishes with a final position error around **1 cm**, so any huge discrepancy is a strong indicator of a bug in the MATLAB code or its configuration.

All example paths assume results are written to the shared `results/` directory used by both MATLAB and Python.

## 1. Detecting the Root Cause

1. **Plot the NED position error** over time. Compare the IMU-derived trajectory against GNSS ground truth to see when the drift begins and whether it grows linearly or quadratically.

   ```matlab
   load('results/Task5_results_IMU_X001_GNSS_X001.mat', ...
        'pos_ned', 'pos_gnss', 't_imu');
   figure;
   plot(t_imu, pos_ned - pos_gnss);
   title('Position error (IMU − GNSS)');
   legend({'North','East','Down'}); grid on;
   ```

2. **Analyse velocity during known static periods.** Non-zero values indicate the Zero Velocity Update (ZUPT) logic is ineffective.

   ```matlab
   static_vel = vel_ned(283:480030, :);
   fprintf('mean vel = [%g %g %g]\n', mean(static_vel));
   fprintf('std  vel = [%g %g %g]\n', std(static_vel));
   ```

3. **Check the biases** produced in Task 2 against those loaded in Task 5. Large differences or zeros signal an issue with the initial static interval.

4. **Review the ZUPT detection thresholds.** Ensure the acceleration and gyro variance limits actually flag stationary segments.

5. **Inspect the integration step.** Using simple Euler integration or forgetting to remove gravity quickly leads to enormous drift.

## 2. Solutions

- **Bias correction.** Scale the gravity vector from the detected static interval to **9.81 m/s²** and compute accelerometer and gyro biases from that same interval.

- **Improved integration.** Rotate the corrected accelerometer data to the NED frame, subtract gravity and integrate using the trapezoidal rule.

- **Robust ZUPT.** Tune the variance thresholds and reset the Kalman filter velocity state to zero whenever a static period is detected.

- **Metrics.** After Task 5 finishes, print RMSE and the final position error so regressions are easy to spot.

## 3. Validation

Re-run `main` for the TRIAD, Davenport and SVD methods. The final position error should drop from tens of kilometres to centimetres, matching the Python results.
