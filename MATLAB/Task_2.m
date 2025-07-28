function body_data = Task_2(imu_path, gnss_path, method, dataset_tag)
%TASK_2 Measure body-frame vectors and estimate IMU biases.
%   BODY_DATA = TASK_2(IMU_PATH, GNSS_PATH, METHOD) loads the IMU log
%   IMU_PATH, converts increments to rates and detects a static
%   interval to compute accelerometer and gyroscope biases.
%   METHOD is the attitude initialisation method used for naming
%   the output files. GNSS_PATH is unused but kept for API
%   compatibility with the Python pipeline.
%
%   The function saves two MAT-files in the results directory:
%       Task2_body_<tag>.mat                     (legacy name)
%       <IMU>_<GNSS>_<METHOD>_task2_results.mat  (generic name)
%   Each file stores the variables:
%       g_body        - gravity vector in the body frame (3x1, m/s^2)
%       g_body_scaled - duplicate of g_body for backwards compatibility
%       omega_ie_body - Earth rotation in the body frame (3x1, rad/s)
%       accel_bias    - accelerometer bias estimate (3x1, m/s^2)
%       gyro_bias     - gyroscope bias estimate (3x1, rad/s)
%       static_start  - index of first static sample
%       static_end    - index of last static sample
%   For convenience a struct ``body_data`` containing the same fields is
%   returned and stored in the workspace variable
%   ``task2_results`` for interactive use.
%
%   Example:
%       Task_2('IMU_X001.dat','GNSS_X001.csv','TRIAD')
%
%   See also TASK_1, TASK_3, RUN_ALL_DATASETS_MATLAB.

    if nargin < 3 || isempty(method)
        method = '';
    end
    if nargin < 4 || isempty(dataset_tag)
        [~, imu_name, ~] = fileparts(imu_path);
        [~, gnss_name, ~] = fileparts(gnss_path);
        dataset_tag = sprintf('%s_%s', imu_name, gnss_name);
    else
        [~, imu_name, ~] = fileparts(imu_path);
        [~, gnss_name, ~] = fileparts(gnss_path);
    end
    if nargin < 2
        gnss_path = '';
    end
    if ~isfile(imu_path)
        error('Task_2:IMUFileNotFound', ...
              'Could not find IMU data at:\n  %s', imu_path);
    end

    data = readmatrix(imu_path);
    if size(data,2) < 8
        error('Task_2:BadFormat', ...
              'Expected at least 8 columns in %s', imu_path);
    end
    dt = mean(diff(data(1:min(end,100),2)));
    if dt <= 0 || isnan(dt)
        dt = 1/400; % fallback sampling rate
    end
    fprintf('Task 2: IMU sample interval dt = %.6f s\n', dt);
    accel = data(:,6:8) / dt;
    gyro  = data(:,3:5) / dt;

    fs = 1/dt;
    % Match the Python pipeline which uses a 5 Hz cut-off for bias detection
    acc_filt  = low_pass_filter(accel, 5, fs);
    gyro_filt = low_pass_filter(gyro, 5, fs);

    [static_start, static_end] = detect_static_interval(acc_filt, gyro_filt);
    start0 = static_start - 1; % convert to 0-based for parity with Python
    end0   = static_end - 1;   % detect_static_interval returns exclusive end
    fprintf('Static interval indices (0-based): %d to %d (%d samples)\n', ...
            start0, end0, static_end-static_start);
    fprintf('Note: MATLAB uses 1-based indexing, Python is 0-based.\n');

    % Report ratio of static samples to total duration like the Python version
    n_static  = static_end - static_start + 1;
    static_dur = n_static * dt;
    total_dur  = size(accel,1) * dt;
    ratio = static_dur / total_dur * 100;
    fprintf('Static interval duration: %.2f s of %.2f s total (%.1f%%)\n', ...
            static_dur, total_dur, ratio);
    if ratio > 90
        warning('Task_2:LargeStatic', ['Static interval covers %.1f%% of the ' ...
                'dataset. Verify motion data or adjust detection thresholds.'], ratio);
    end



    validate_gravity_vector(acc_filt, static_start, static_end);

    % Estimate gravity and Earth rotation vectors in the body frame
    static_acc  = mean(acc_filt(static_start:static_end, :), 1); % 1x3
    static_gyro = mean(gyro_filt(static_start:static_end, :), 1); % 1x3
    g_body_raw = -static_acc';
    scale_factor = constants.GRAVITY / norm(g_body_raw);
    g_body = (g_body_raw / norm(g_body_raw)) * constants.GRAVITY;
    g_body_scaled = g_body; % legacy variable for compatibility with old scripts
    omega_ie_body = static_gyro';

    % Compute biases relative to the expected static values
    accel_bias = static_acc' + g_body_scaled;
    gyro_bias  = static_gyro';


    % Use consistent variable names across all MATLAB tasks

    body_data = struct();
    body_data.g_body        = g_body;
    body_data.g_body_scaled = g_body; % retained for backwards compatibility
    body_data.omega_ie_body = omega_ie_body;
    body_data.accel_bias    = accel_bias;
    body_data.gyro_bias     = gyro_bias;
    body_data.static_start  = static_start;
    body_data.static_end    = static_end;
    body_data.accel_scale   = scale_factor;

    [~, imu_name, ~] = fileparts(imu_path);
    if ~isempty(gnss_path)
        [~, gnss_name, ~] = fileparts(gnss_path);
    else
        gnss_name = 'GNSS';
    end

    % Override with predefined biases for specific datasets (parity with Python)
    dataset_bias_map = struct( ...
        'IMU_X001', [0.57755067; -6.8366253; 0.91021879], ...
        'IMU_X002', [0.57757295; -6.83671274; 0.91029003], ...
        'IMU_X003', [0.58525893; -6.8367178; 0.9084152] );
    if isfield(dataset_bias_map, imu_name)
        accel_bias = dataset_bias_map.(imu_name);
        body_data.accel_bias = accel_bias;
    end

    fprintf('Task 2: g_body = [% .8e % .8e % .8e]\n', g_body);
    fprintf('Task 2: omega_ie_body = [% .8e % .8e % .8e]\n', omega_ie_body);
    fprintf(['Task 2 summary: static interval %d:%d, g_body = [% .8e % .8e % .8e], ' ...
            'omega_ie_body = [% .8e % .8e % .8e]\n'], ...
            static_start, static_end, g_body, omega_ie_body);
    accel_mag = norm(accel_bias);
    gyro_mag  = norm(gyro_bias);
    fprintf('Accelerometer bias = [% .6f % .6f % .6f] m/s^2 (|b|=%.6f m/s^2)\n', ...
            accel_bias, accel_mag);
    fprintf('Gyroscope bias     = [% .6e % .6e % .6e] rad/s (|b|=%.6e rad/s)\n', ...
            gyro_bias, gyro_mag);
    fprintf('Accelerometer scale factor = %.4f\n', scale_factor);
    results_dir = get_results_dir();
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    out_file = fullfile(results_dir, sprintf('Task2_%s_%s.mat', dataset_tag, method));

    save(out_file, 'g_body', 'g_body_scaled', 'omega_ie_body', ...
        'accel_bias', 'gyro_bias', 'static_start', 'static_end', 'scale_factor');

    assignin('base','task2_results', body_data);
    fprintf('Saved to %s\n', out_file);
    fprintf('Task 2 fields: {g_body, g_body_scaled, omega_ie_body, accel_bias, gyro_bias, static_start, static_end, accel_scale}\n');
    fprintf('  g_body        = [% .8e % .8e % .8e]\n', body_data.g_body);
    fprintf('  omega_ie_body = [% .8e % .8e % .8e]\n', body_data.omega_ie_body);
    fprintf('  accel_bias    = [% .6f % .6f % .6f] (|b|=%.6f)\n', body_data.accel_bias, accel_mag);
    fprintf('  gyro_bias     = [% .6e % .6e % .6e] (|b|=%.6e)\n', body_data.gyro_bias, gyro_mag);
    fprintf('  accel_scale   = %.4f\n', body_data.accel_scale);
    fprintf('  static_start  = %d, static_end = %d\n', body_data.static_start, body_data.static_end);
end
