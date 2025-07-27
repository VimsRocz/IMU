function Task_2(imu_data, method)
%TASK_2 Estimate IMU biases from a static interval.
%   TASK_2(IMU_DATA, METHOD) low-pass filters the accelerometer and
%   gyroscope data, detects a static interval using DETECT_STATIC_INTERVAL,
%   computes mean biases and validates gravity magnitude.  Results are saved
%   using SAVE_TASK_RESULTS.
%
%   IMU_DATA must be a struct with fields 'accel' and 'gyro'.
%
%   This simplified implementation mirrors the Python preprocessing used in
%   run_triad_only.py.

if nargin < 2, method = 'TRIAD'; end

acc_body = low_pass_filter(imu_data.accel, 10, 100);
gyro_body = low_pass_filter(imu_data.gyro, 10, 100);
[static_start, static_end] = detect_static_interval(acc_body, gyro_body);
[acc_bias, gyro_bias] = compute_biases(acc_body, gyro_body, static_start, static_end);

is_valid = validate_gravity_vector(acc_body, static_start, static_end);
if ~is_valid
    warning('Gravity validation failed');
end

results = struct('acc_bias', acc_bias, 'gyro_bias', gyro_bias, ...
    'static_start', static_start, 'static_end', static_end);
save_task_results(results, 'IMU_X002', 'GNSS_X002', method, 2);
end
