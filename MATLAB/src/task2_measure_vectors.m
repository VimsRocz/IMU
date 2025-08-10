function [acc_bias, gyro_bias, static_start, static_end] = task2_measure_vectors(acc_body, gyro_body, results_dir)
%TASK2_MEASURE_VECTORS Detect static interval and compute IMU biases.
%   [ACC_BIAS, GYRO_BIAS, START, END] = TASK2_MEASURE_VECTORS(ACC_BODY, GYRO_BODY, RESULTS_DIR)
%   detects a static interval in the body-frame accelerometer and gyroscope
%   data using the same parameters as the Python implementation and returns
%   the computed biases. START and END use 1-based indexing with END
%   exclusive. The results are saved to ``Task2_IMU_biases.mat`` in
%   RESULTS_DIR when supplied.
%
%   This function mirrors the behaviour of ``gnss_imu_fusion.init.measure_body_vectors``
%   but operates on already loaded data arrays.
%
%   Example:
%       [acc_b, gyro_b] = task2_measure_vectors(acc, gyro, get_results_dir());
%
%   See also DETECT_STATIC_INTERVAL, COMPUTE_BIASES.

    if nargin < 3 || isempty(results_dir)
        results_dir = get_results_dir();
    end

    [static_start, static_end] = detect_static_interval(acc_body, gyro_body, 80, 0.01, 1e-6, 80);

    [acc_bias, gyro_bias] = compute_biases(acc_body, gyro_body, static_start, static_end);

    fprintf('Static interval: %d to %d\n', static_start, static_end - 1);
    fprintf('Computed accelerometer bias: [%.8f, %.8f, %.8f] m/s^2\n', acc_bias(1), acc_bias(2), acc_bias(3));
    fprintf('Computed gyroscope bias: [%.8e, %.8e, %.8e] rad/s\n', gyro_bias(1), gyro_bias(2), gyro_bias(3));

    if ~exist(results_dir, 'dir'); mkdir(results_dir); end
    save(fullfile(results_dir, 'Task2_IMU_biases.mat'), 'acc_bias', 'gyro_bias', 'static_start', 'static_end');
end
