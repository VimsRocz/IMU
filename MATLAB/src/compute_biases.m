function [acc_bias, gyro_bias] = compute_biases(acc_body, gyro_body, static_start, static_end)
%COMPUTE_BIASES Estimate accelerometer and gyroscope biases.
%   [ACC_BIAS, GYRO_BIAS] = COMPUTE_BIASES(ACC_BODY, GYRO_BODY, START, END)
%   returns the mean accelerometer and gyroscope measurements over the
%   half-open static interval ``[START, END)``.  This mirrors the Python
%   implementation used in ``gnss_imu_fusion.measure_body_vectors``.
%
%   Inputs are assumed to be in body frame units of m/s^2 and rad/s.
%   START and END are 1-based indices selecting a continuous static block
%   where END is exclusive.
%
%   Example:
%       [acc_bias, gyro_bias] = compute_biases(acc, gyro, 1, 4000);
%
%   See also DETECT_STATIC_INTERVAL.

static_acc  = acc_body(static_start:static_end-1, :);
static_gyro = gyro_body(static_start:static_end-1, :);
acc_bias = mean(static_acc, 1);
gyro_bias = mean(static_gyro, 1);
end
