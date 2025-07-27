function body_data = Task_2(imu_path, gnss_path, method)
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
%   Both files contain a struct named ``body_data`` with these fields:
%       g_body        - gravity vector in the body frame (3x1, m/s^2)
%       g_body_scaled - same as g_body for compatibility
%       omega_ie_body - Earth rotation in the body frame (3x1, rad/s)
%       accel_bias    - accelerometer bias estimate (3x1, m/s^2)
%       acc_bias      - alias of accel_bias for older code
%       gyro_bias     - gyroscope bias estimate (3x1, rad/s)
%       static_start  - index of first static sample
%       static_end    - index of last static sample
%   The struct is returned and also stored in the workspace variable
%   ``task2_results`` for interactive use.
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

    data = readmatrix(imu_path);
    if size(data,2) < 8
        error('Task_2:BadFormat', ...
              'Expected at least 8 columns in %s', imu_path);
    end
    dt = mean(diff(data(1:min(end,100),2)));
    if dt <= 0 || isnan(dt)
        dt = 1/400; % fallback sampling rate
    end
    accel = data(:,6:8) / dt;
    gyro  = data(:,3:5) / dt;

    fs = 1/dt;
    acc_filt  = low_pass_filter(accel, 10, fs);
    gyro_filt = low_pass_filter(gyro, 10, fs);

    [static_start, static_end] = detect_static_interval(acc_filt, gyro_filt);
    [acc_bias, gyro_bias] = compute_biases(acc_filt, gyro_filt, ...
                                           static_start, static_end);

    validate_gravity_vector(acc_filt, static_start, static_end);

    % Estimate gravity and Earth rotation vectors in the body frame
    static_acc  = mean(acc_filt(static_start:static_end, :), 1); % 1x3
    static_gyro = mean(gyro_filt(static_start:static_end, :), 1); % 1x3
    g_body_raw = -static_acc';
    g_body = (g_body_raw / norm(g_body_raw)) * constants.GRAVITY;
    omega_ie_body = static_gyro';

    body_data = struct();
    body_data.g_body       = g_body;
    body_data.g_body_scaled= g_body; % kept for compatibility
    body_data.omega_ie_body= omega_ie_body;
    body_data.accel_bias   = acc_bias';
    body_data.acc_bias     = acc_bias';
    body_data.gyro_bias    = gyro_bias';
    body_data.static_start = static_start;
    body_data.static_end   = static_end;

    [~, imu_name, ~] = fileparts(imu_path);
    if ~isempty(gnss_path)
        [~, gnss_name, ~] = fileparts(gnss_path);
    else
        gnss_name = 'GNSS';
    end
    pair_tag = [imu_name '_' gnss_name];
    if isempty(method)
        tag = pair_tag;
        method_tag = 'AllMethods';
    else
        tag = [pair_tag '_' method];
        method_tag = method;
    end

    results_dir = get_results_dir();
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    legacy_file = fullfile(results_dir, ['Task2_body_' tag '.mat']);
    generic_file = fullfile(results_dir, ...
        sprintf('%s_%s_%s_task2_results.mat', imu_name, gnss_name, method_tag));
    save(legacy_file, 'body_data');
    save(generic_file, 'body_data');

    assignin('base','task2_results', body_data);
    fprintf('Task 2 results saved to %s\n', legacy_file);
    fprintf('Task 2 fields:\n');
    disp(fieldnames(body_data));
end
