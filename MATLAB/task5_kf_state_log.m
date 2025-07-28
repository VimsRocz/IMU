function task5_kf_state_log()
%TASK5_KF_STATE_LOG Example 15-state Kalman Filter loop storing x_log.
%   This simplified demonstration mirrors the Python stub
%   ``task5_kf_state_log.py``. It pre-allocates a history matrix, runs a
%   dummy filter loop and saves ``x_log`` under ``MATLAB/results``.
%
%   Usage:
%       task5_kf_state_log

    fprintf('--- Starting Task 5: Sensor Fusion with Kalman Filter ---\n');

    % Configure Kalman Filter (placeholder setup)
    num_states = 15;      % 15-state filter
    num_samples = 500000; % Total IMU samples
    x = zeros(num_states, 1);

    fprintf('Task 5: Initializing state history matrix...\n');
    x_log = zeros(num_states, num_samples);
    fprintf('Task 5: x_log initialized with size %dx%d\n', num_states, num_samples);

    fprintf('Task 5: Starting Kalman Filter loop over %d IMU samples...\n', num_samples);
    for t = 1:num_samples
        % Placeholder for prediction and update steps
        x_log(:, t) = x;  %#ok<SAGROW> - example store
        if mod(t, 100000) == 0
            fprintf('Task 5: Processed sample %d/%d\n', t, num_samples);
        end
    end
    fprintf('Task 5: Completed state logging for %d samples\n', num_samples);

    % Save results under repository results directory
    results_dir = get_results_dir();
    if ~exist(results_dir, 'dir'); mkdir(results_dir); end
    results_file = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task5_results.mat');
    fprintf('Task 5: Saving results to %s...\n', results_file);
    if isfile(results_file)
        data = load(results_file);
        data.x_log = x_log; %#ok<STRNU>
        save(results_file, '-struct', 'data');
    else
        save(results_file, 'x_log');
    end
    fprintf('Task 5: State history (x_log) saved to %s\n', results_file);

    % Verify save
    try
        check = load(results_file, 'x_log');
        fprintf('Task 5: Verified x_log saved, size: %dx%d\n', size(check.x_log));
    catch
        warning('Task 5: Failed to verify x_log save in %s', results_file);
    end
    fprintf('Task 5: Completed successfully\n');
end
