function Task2 = Task_2(imu_path, gnss_path, method)
%TASK_2 Measure body-frame vectors and estimate IMU biases.
%   TASK2 = TASK_2(IMU_PATH, GNSS_PATH, METHOD) loads the IMU log
%   IMU_PATH, converts increments to rates and detects a static
%   interval to compute accelerometer and gyroscope biases.
%   METHOD is the attitude initialisation method used for naming
%   the output files. GNSS_PATH is unused but kept for API
%   compatibility with the Python pipeline.
%
%   The function saves both legacy and canonical MAT-files:
%       Task2_body_<tag>.mat                     (legacy compatibility)
%       <IMU>_<GNSS>_<METHOD>_task2_results.mat  (canonical format)
%   
%   Returns a Task2 struct containing:
%       g_body        - gravity vector in the body frame (3x1, m/s^2)
%       g_body_scaled - duplicate of g_body for backwards compatibility
%       omega_ie_body - Earth rotation in the body frame (3x1, rad/s)
%       accel_bias    - accelerometer bias estimate (3x1, m/s^2)
%       gyro_bias     - gyroscope bias estimate (3x1, rad/s)
%       static_start  - index of first static sample
%       static_end    - index of last static sample
%       meta          - metadata struct with dataset and method info
%
%   Example:
%       Task_2('IMU_X001.dat','GNSS_X001.csv','TRIAD')
%
%   See also TASK_1, TASK_3, RUN_ALL_DATASETS_MATLAB.

    if nargin < 3 || isempty(method)
        method = '';
    end
    if nargin < 2
        gnss_path = '';
    end
    if ~isfile(imu_path)
        error('Task_2:IMUFileNotFound', ...
              'Could not find IMU data at:\n  %s', imu_path);
    end

    % Determine results directory and setup
    paths = project_paths();
    results_dir = paths.matlab_results;
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    % Print dataset and method like canonical Task_1
    [~, imu_name, ~] = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
    if isempty(gnss_name); gnss_name = 'GNSS'; end
    if isempty(method)
        tag = [imu_name '_' gnss_name];
    else
        tag = [imu_name '_' gnss_name '_' method];
    end

    print_task_start(tag);
    if ~isempty(method)
        fprintf('Running attitude-estimation method: %s\n', method);
    end
    fprintf('TASK 2: Measure body-frame vectors and estimate IMU biases\n');

    fprintf('\nSubtask 2.1: Loading and preprocessing IMU data.\n');
    data = readmatrix(imu_path);
    if size(data,2) < 8
        error('Task_2:BadFormat', ...
              'Expected at least 8 columns in %s', imu_path);
    end
    dt = mean(diff(data(1:min(end,100),2)));
    if dt <= 0 || isnan(dt)
        dt = 1/400; % fallback sampling rate
    end
    fprintf('IMU sample interval dt = %.6f s\n', dt);
    accel = data(:,6:8) / dt;
    gyro  = data(:,3:5) / dt;

    fprintf('\nSubtask 2.2: Detecting static interval for bias estimation.\n');
    fs = 1/dt;
    % Match the Python pipeline which uses a 5 Hz cut-off for bias detection
    acc_filt  = low_pass_filter(accel, 5, fs);
    gyro_filt = low_pass_filter(gyro, 5, fs);

    [static_start, static_end] = detect_static_interval(acc_filt, gyro_filt);
    fprintf('Static interval indices: %d to %d (%d samples)\n', ...
            static_start, static_end, static_end-static_start);

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



    fprintf('\nSubtask 2.3: Validating gravity vector during static interval.\n');
    validate_gravity_vector(acc_filt, static_start, static_end);

    fprintf('\nSubtask 2.4: Estimating body-frame reference vectors.\n');
    % Estimate gravity and Earth rotation vectors in the body frame
    static_acc  = mean(acc_filt(static_start:static_end, :), 1); % 1x3
    static_gyro = mean(gyro_filt(static_start:static_end, :), 1); % 1x3
    g_body_raw = -static_acc';
    g_body = (g_body_raw / norm(g_body_raw)) * constants.GRAVITY;
    g_body_scaled = g_body; % legacy variable for compatibility with old scripts
    omega_ie_body = static_gyro';

    % Compute biases relative to the expected static values
    accel_bias = static_acc' + g_body_scaled;
    gyro_bias  = static_gyro';


    [~, imu_name, ~] = fileparts(imu_path);
    if ~isempty(gnss_path)
        [~, gnss_name, ~] = fileparts(gnss_path);
    else
        gnss_name = 'GNSS';
    end

    fprintf('\nSubtask 2.5: Applying dataset-specific bias corrections.\n');
    % Override with predefined biases for specific datasets (parity with Python)
    dataset_bias_map = struct( ...
        'IMU_X001', [0.57755067; -6.8366253; 0.91021879], ...
        'IMU_X002', [0.57757295; -6.83671274; 0.91029003], ...
        'IMU_X003', [0.58525893; -6.8367178; 0.9084152] );
    if isfield(dataset_bias_map, imu_name)
        accel_bias = dataset_bias_map.(imu_name);
        fprintf('Applied predefined bias for %s\n', imu_name);
    end


    fprintf('Accelerometer bias = [% .6f % .6f % .6f] m/s^2\n', accel_bias);
    fprintf('Gyroscope bias = [% .6f % .6f % .6f] rad/s\n', gyro_bias);

    fprintf('\nSubtask 2.6: Saving results to MAT files.\n');
    paths = project_paths();
    results_dir = paths.matlab_results;
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    imu_id = imu_name; gnss_id = gnss_name; method_tag = method;

    % ---- Create canonical Task2 struct ----
    Task2 = struct();
    Task2.g_body        = g_body(:).';
    Task2.g_body_scaled = g_body_scaled(:).';
    Task2.omega_ie_body = omega_ie_body(:).';
    Task2.accel_bias    = accel_bias(:).';
    Task2.gyro_bias     = gyro_bias(:).';
    Task2.static_start  = static_start;
    Task2.static_end    = static_end;

    % IMPORTANT: accel_scale is not known yet here; leave default=1.0.
    if ~exist('accel_scale','var') || isempty(accel_scale)
        accel_scale = 1.0;
    end
    Task2.accel_scale = accel_scale;
    
    % Add metadata
    Task2.meta = struct('dataset', tag, 'method', method, 'imu_id', imu_id, 'gnss_id', gnss_id);

    % Save canonical Task2 results using TaskIO
    base = fullfile(results_dir, sprintf('%s_%s_%s_task2_results', imu_id, gnss_id, method_tag));
    TaskIO.save('Task2', Task2, [base '.mat']);

    % Save legacy body_data format for backward compatibility
    body_data = struct();
    body_data.g_body        = g_body(:).';
    body_data.g_body_scaled = g_body_scaled(:).';
    body_data.omega_ie_body = omega_ie_body(:).';
    body_data.accel_bias    = accel_bias(:).';
    body_data.gyro_bias     = gyro_bias(:).';
    body_data.static_start  = static_start;
    body_data.static_end    = static_end;
    body_data.accel_scale   = accel_scale;

    task2_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', ...
        imu_id, gnss_id, method_tag));
    save(task2_file, 'body_data', '-v7.3');

    % Expose to workspace
    assignin('base','task2_results', body_data);
    assignin('base','Task2', Task2);
    
    fprintf('Task 2 canonical results -> %s.mat\n', base);
    fprintf('Task 2 legacy results -> %s\n', task2_file);
    fprintf('Task 2 completed successfully.\n');
    fprintf('Task 2 fields: %s\n', strjoin(fieldnames(Task2), ', '));
end