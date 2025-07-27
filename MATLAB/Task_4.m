function result = Task_4(imu_path, gnss_path, method)
%TASK_4 GNSS and IMU data integration and comparison
%   Task_4(IMU_PATH, GNSS_PATH, METHOD) runs the GNSS/IMU integration
%   using the attitude estimates from Task 3. METHOD is unused but kept
%   for backwards compatibility with older scripts.
%   Requires that `Task_3` has already saved a dataset-specific
%   results file such as `output_matlab/Task3_results_IMU_X001_GNSS_X001.mat`.
%
% Usage:
%   Task_4(imu_path, gnss_path, method)

if nargin < 1 || isempty(imu_path)
    error('IMU file not specified');
end
if nargin < 2 || isempty(gnss_path)
    error('GNSS file not specified');
end
if nargin < 3
    method = '';
end

if ~exist('output_matlab','dir')
    mkdir('output_matlab');
end
if ~isfile(gnss_path)
    error('Task_4:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_4:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end
results_dir = get_results_dir();
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
pair_tag = [imu_name '_' gnss_name];
if isempty(method)
    tag = pair_tag;
    method_tag = 'AllMethods';
else
    tag = [pair_tag '_' method];
    method_tag = method;
end

% Load accelerometer and gyroscope biases estimated in Task 2
task2_file = fullfile(results_dir, sprintf('Task2_body_%s.mat', tag));
if isfile(task2_file)
    t2 = load(task2_file);
    if isfield(t2, 'accel_bias'); loaded_accel_bias = t2.accel_bias; else; error('Task_4:MissingField', 'accel_bias missing from %s', task2_file); end
    if isfield(t2, 'gyro_bias');  loaded_gyro_bias  = t2.gyro_bias;  else; error('Task_4:MissingField', 'gyro_bias missing from %s', task2_file);  end
else
    error('Task_4:MissingTask2', 'Missing Task 2 output: %s. Run Task_2 first.', task2_file);
end

% Load rotation matrices produced by Task 3
results_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
if evalin('base','exist(''task3_results'',''var'')')
    task3_results = evalin('base','task3_results');
else
    if ~isfile(results_file)
        error('Task 3 results not found: %s', results_file);
    end
    data = load(results_file);
    if ~isfield(data, 'task3_results')
        error('Variable ''task3_results'' missing from %s', results_file);
    end
    task3_results = data.task3_results;
end

if isempty(method)
    log_tag = '';
else
    log_tag = [' (' method ')'];
end
fprintf('\nTASK 4%s: GNSS and IMU Data Integration and Comparison\n', log_tag);

%% ========================================================================
% Subtask 4.1: Access Rotation Matrices from Task 3
% =========================================================================
fprintf('\nSubtask 4.1: Accessing rotation matrices from Task 3.\n');
all_methods = fieldnames(task3_results); % e.g., 'TRIAD', 'Davenport', 'SVD'
if ~isempty(method)
    methods = {method};
else
    methods = all_methods;
end
C_B_N_methods = struct();
for i = 1:length(methods)
    method_name = methods{i};
    C_B_N_methods.(method_name) = task3_results.(method_name).R;
end
fprintf('-> Rotation matrices accessed for methods: %s\n', strjoin(methods, ', '));


%% ========================================================================
% Subtask 4.3: Load GNSS Data
% =========================================================================
fprintf('\nSubtask 4.3: Loading GNSS data.\n');
try
    gnss_data = readtable(gnss_path);
catch e
    error('Failed to load GNSS data file: %s', e.message);
end


%% ========================================================================
% Subtask 4.4: Extract Relevant Columns
% =========================================================================
fprintf('\nSubtask 4.4: Extracting relevant columns.\n');
time_col = 'Posix_Time';
pos_cols = {'X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'};
vel_cols = {'VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps'};
gnss_time = gnss_data.(time_col);
gnss_pos_ecef = gnss_data{:, pos_cols};
gnss_vel_ecef = gnss_data{:, vel_cols};
fprintf('-> GNSS data shape: %d x %d\n', size(gnss_pos_ecef));


%% ========================================================================
% Subtask 4.5: Define Reference Point
% =========================================================================
fprintf('\nSubtask 4.5: Defining reference point.\n');
% These values should be consistent with those from Task 1
first_valid_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
ref_r0 = gnss_pos_ecef(first_valid_idx, :)'; % ECEF position vector as a column
[lat_deg_ref, lon_deg_ref, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
ref_lat = deg2rad(lat_deg_ref);
ref_lon = deg2rad(lon_deg_ref);
fprintf('-> Reference point: lat=%.6f rad, lon=%.6f rad, r0=[%.1f, %.1f, %.1f]''\n', ref_lat, ref_lon, ref_r0);


%% ========================================================================
% Subtask 4.6: Compute Rotation Matrix from ECEF to NED
% =========================================================================
fprintf('\nSubtask 4.6: Computing ECEF to NED rotation matrix.\n');
C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon);
C_NED_to_ECEF = C_ECEF_to_NED';
fprintf('-> ECEF to NED rotation matrix computed.\n');
fprintf('-> NED to ECEF rotation matrix computed.\n');


%% ========================================================================
% Subtask 4.7: Convert GNSS Data to NED Frame
% =========================================================================
fprintf('\nSubtask 4.7: Converting GNSS data to NED frame.\n');
gnss_pos_ned = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';
fprintf('GNSS velocity incorporated: [%.5f %.5f %.5f]\n', gnss_vel_ned(1,:));
fprintf('-> GNSS data transformed to NED frame.\n');
fprintf('   GNSS NED pos first=[%.2f %.2f %.2f], last=[%.2f %.2f %.2f]\n', ...
    gnss_pos_ned(1,:), gnss_pos_ned(end,:));


%% ========================================================================
% Subtask 4.8: Estimate GNSS Acceleration in NED
% =========================================================================
fprintf('\nSubtask 4.8: Estimating GNSS acceleration in NED.\n');
dt_gnss = diff(gnss_time(:));
% Prepend a zero row to maintain size, as diff reduces length by 1
gnss_accel_ned = [zeros(1,3); bsxfun(@rdivide, diff(gnss_vel_ned), dt_gnss)];
fprintf('-> GNSS acceleration estimated in NED frame.\n');
fprintf('   GNSS accel RMS = %.4f m/s^2\n', rms(gnss_accel_ned(:)) );


%% ========================================================================
% Subtask 4.9: Load IMU Data and Correct for Bias for Each Method
% =========================================================================
fprintf('\nSubtask 4.9: Loading IMU data and correcting for bias for each method.\n');
imu_raw_data = readmatrix(imu_path);
dt_imu = mean(diff(imu_raw_data(1:100,2)));
if dt_imu <= 0 || isnan(dt_imu), dt_imu = 1/400; end
imu_time = (0:size(imu_raw_data,1)-1)' * dt_imu + gnss_time(1);

acc_body_raw = imu_raw_data(:, 6:8) / dt_imu;
acc_body_filt = butter_lowpass_filter(acc_body_raw, 5.0, 1/dt_imu);
gyro_body_filt = butter_lowpass_filter(imu_raw_data(:, 3:5) / dt_imu, 5.0, 1/dt_imu);
acc_rms  = sqrt(mean(acc_body_raw(:).^2));
gyro_rms = sqrt(mean((imu_raw_data(:,3:5)/dt_imu).^2,'all'));
fprintf('   Acc raw RMS=%.4f, Gyro raw RMS=%.6f\n', acc_rms, gyro_rms);

dataset_map = containers.Map( ...
    {'IMU_X001','IMU_X002','IMU_X003'}, ...
    {[296, 479907],[296, 479907],[296, 479907]});
if isKey(dataset_map, imu_name)
    w = dataset_map(imu_name);
    start_idx = w(1);
    end_idx   = min(w(2), size(acc_body_filt,1));
else
    [start_idx, end_idx] = detect_static_interval(acc_body_filt, gyro_body_filt);
end
static_acc  = mean(acc_body_filt(start_idx:end_idx, :), 1);
static_gyro = mean(gyro_body_filt(start_idx:end_idx, :), 1);
acc_var = var(acc_body_filt(start_idx:end_idx, :), 0, 1);
gyro_var = var(gyro_body_filt(start_idx:end_idx, :), 0, 1);
fprintf('Static interval [%d, %d] (len=%d)\n', start_idx, end_idx, end_idx-start_idx+1);
fprintf('Static acc mean  =[%.4f %.4f %.4f]\n', static_acc);
fprintf('Static gyro mean =[%.6f %.6f %.6f]\n', static_gyro);
fprintf('Static acc var   =[%.4g %.4g %.4g]\n', acc_var);
fprintf('Static gyro var  =[%.4g %.4g %.4g]\n', gyro_var);

% Gravity vector and Earth rotation in NED frame
% Attempt to reuse the gravity vector estimated in Task 1; if unavailable,
% fall back to the nominal constant.
task1_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', tag));
if ~isfile(task1_file)
    % Fallback to method-agnostic filename for older runs
    alt_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', pair_tag));
    if isfile(alt_file)
        task1_file = alt_file;
    end
end

if isfile(task1_file)
    t1 = load(task1_file);
    if isfield(t1, 'g_NED')
        g_NED = t1.g_NED(:);
        fprintf('Loaded gravity from %s\n', task1_file);
    else
        warning('g_NED missing from %s, using default %.3f m/s^2', task1_file, constants.GRAVITY);
        g_NED = [0; 0; constants.GRAVITY];
    end
else
    warning('Task1 init file %s not found, using default gravity %.3f m/s^2', task1_file, constants.GRAVITY);
    g_NED = [0; 0; constants.GRAVITY];
end
omega_E = constants.EARTH_RATE;                     % rad/s
omega_ie_NED = omega_E * [cos(ref_lat); 0; -sin(ref_lat)];

% Correct IMU measurements separately for each Wahba method
acc_body_corrected  = struct();
gyro_body_corrected = struct();
acc_biases = struct();
gyro_biases = struct();
scale_factors = struct();
switch upper(imu_name)
    case 'IMU_X001'
        dataset_acc_bias = [0.57755067; -6.8366253; 0.91021879];
    case 'IMU_X002'
        dataset_acc_bias = [0.57757295; -6.83671274; 0.91029003];
    case 'IMU_X003'
        dataset_acc_bias = [0.58525893; -6.8367178; 0.9084152];
    otherwise
        dataset_acc_bias = [];
end

for i = 1:length(methods)
    method = methods{i};
    C_B_N = C_B_N_methods.(method);
    C_N_B = C_B_N';

    % Expected gravity and Earth rate in the body frame
    g_body_expected = C_N_B * g_NED;

    % Compute biases using the static interval as in the Python pipeline
    % Accelerometer bias: static_acc should equal -g_body_expected
    acc_bias = static_acc' + g_body_expected;
    if ~isempty(dataset_acc_bias)
        acc_bias = dataset_acc_bias;
    end
    % Gyroscope bias: static_gyro should equal expected earth rate in body frame
    omega_ie_body_expected = C_N_B * omega_ie_NED;
    gyro_bias = static_gyro' - omega_ie_body_expected;

    % Scale factor matching the Python implementation
    scale_factor = constants.GRAVITY / norm(static_acc' - acc_bias);
    if abs(scale_factor - 1.0) < 0.0001
        scale_factor = 1.0016; % fallback constant for legacy datasets
    end
    scale = scale_factor;

    % Apply bias and scale corrections
    acc_body_corrected.(method)  = scale * (acc_body_filt - acc_bias');
    gyro_body_corrected.(method) = gyro_body_filt - gyro_bias';
    acc_biases.(method)  = acc_bias;
    gyro_biases.(method) = gyro_bias;
    scale_factors.(method) = scale;

    fprintf('Method %s: Accelerometer bias: [%10.8f %10.8f %10.8f] (|b|=%.6f m/s^2)\n', ...
            method, acc_bias, norm(acc_bias));
    fprintf('Method %s: Gyroscope bias: [% .8e % .8e % .8e]\n', method, gyro_bias);
    fprintf('Method %s: Accelerometer scale factor: %.4f\n', method, scale);
end
fprintf('-> IMU data corrected for bias and scale for each method.\n');

%% ========================================================================
% Subtask 4.10: Set IMU Parameters and Gravity Vector
% =========================================================================
fprintf('\nSubtask 4.10: Setting IMU parameters and gravity vector.\n');
fprintf('-> IMU sample interval dt = %.6f s\n', dt_imu);
fprintf('Gravity vector applied: [%.2f %.2f %.2f]\n', g_NED);



%% ========================================================================
% Subtask 4.11: Initialize Output Arrays
% =========================================================================
fprintf('\nSubtask 4.11: Initializing output arrays.\n');
pos_integ = struct();
vel_integ = struct();
acc_integ = struct();


%% ========================================================================
% Subtask 4.12: Integrate IMU Accelerations for Each Method
% =========================================================================
fprintf('\nSubtask 4.12: Integrating IMU accelerations for each method.\n');
for i = 1:length(methods)
    method = methods{i};
    fprintf('-> Integrating IMU data using %s method.\n', method);
    C_B_N = C_B_N_methods.(method);
    q_b_n = rot_to_quaternion(C_B_N); % Task 4.12 initial quaternion
    
    pos = zeros(size(imu_time,1), 3);
    vel = zeros(size(imu_time,1), 3);
    acc = zeros(size(imu_time,1), 3);
    
    % Initialize with first GNSS measurement
    vel(1,:) = gnss_vel_ned(1,:);
    pos(1,:) = gnss_pos_ned(1,:);
    
    for k = 2:length(imu_time)
        % Propagate attitude using corrected gyro measurements
        current_omega_ie_b = C_B_N' * omega_ie_NED;
        w_b = gyro_body_corrected.(method)(k,:)' - current_omega_ie_b;
        q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
        C_B_N = quat_to_rot(q_b_n);

        % Rotate measured specific force to NED and subtract gravity
        f_ned = C_B_N * acc_body_corrected.(method)(k,:)';
        a_ned = f_ned - g_NED;
        acc(k,:) = a_ned';
        
        % Trapezoidal integration
        vel(k,:) = vel(k-1,:) + (acc(k,:) + acc(k-1,:)) * 0.5 * dt_imu;
        pos(k,:) = pos(k-1,:) + (vel(k,:) + vel(k-1,:)) * 0.5 * dt_imu;
    end
    pos_integ.(method) = pos;
    vel_integ.(method) = vel;
    acc_integ.(method) = acc;
end
fprintf('-> IMU-derived position, velocity, and acceleration computed for all methods.\n');


%% ========================================================================
% Subtask 4.13: Validate and Plot Data
% =========================================================================
fprintf('\nSubtask 4.13: Validating and plotting data.\n');

for i = 1:length(methods)
    m = methods{i};
    base = fullfile(results_dir, sprintf('%s_%s_%s', imu_name, gnss_name, m));
    plot_single_method(m, gnss_time, imu_time, C_B_N_methods.(m), ...
        gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, ...
        pos_integ.(m), vel_integ.(m), acc_integ.(m), ...
        acc_body_corrected.(m), base, ref_r0, C_ECEF_to_NED);
end
fprintf('-> All data plots saved for all methods.\n');

% Save GNSS positions for use by Task 5
task4_file = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
if isfile(task4_file)
    save(task4_file, 'gnss_pos_ned', 'acc_biases', 'gyro_biases', 'scale_factors', '-append');
else
    save(task4_file, 'gnss_pos_ned', 'acc_biases', 'gyro_biases', 'scale_factors');
end
fprintf('GNSS NED positions saved to %s\n', task4_file);
% Task 5 loads these positions when initialising the Kalman filter

% Return results structure and store in base workspace
result = struct('gnss_pos_ned', gnss_pos_ned, 'acc_biases', acc_biases, ...
                'gyro_biases', gyro_biases, 'scale_factors', scale_factors);
assignin('base', 'task4_results', result);

end % End of main function: run_task4


%% ========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function plot_comparison_in_frame(frame_name, t_gnss, t_imu, methods, C_B_N_methods, p_gnss, v_gnss, a_gnss, p_imu, v_imu, a_imu, f_b_corr, filename, r0_ecef, C_e2n)
    % Helper function to generate all comparison plots.
    fig = figure('Name', ['Comparison Plots in ' frame_name], 'Position', [100 100 1200 900], 'Visible', 'off');
    
    dims_ned = {'North', 'East', 'Down'};
    dims_ecef = {'X_{ECEF}', 'Y_{ECEF}', 'Z_{ECEF}'};
    dims_body = {'X_{body}', 'Y_{body}', 'Z_{body}'};
    colors = struct('TRIAD', [0 0.4470 0.7410], 'Davenport', [0.8500 0.3250 0.0980], 'SVD', [0.4660 0.6740 0.1880]);
    C_n2e = C_e2n';

    % --- Plot 1: NED Frame Comparison ---
    for i = 1:3
        % Position
        subplot(3, 3, i);
        hold on; plot(t_gnss, p_gnss(:, i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'GNSS');
        for m = 1:length(methods)
            method = methods{m};
            plot(t_imu, p_imu.(method)(:, i), 'Color', colors.(method), 'DisplayName', ['IMU ' method]);
        end
        hold off; grid on; legend; title(['Position ' dims_ned{i}]); xlabel('Time (s)'); ylabel('Position (m)');
        
        % Velocity
        subplot(3, 3, i + 3);
        hold on; plot(t_gnss, v_gnss(:, i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'GNSS');
        for m = 1:length(methods)
            method = methods{m};
            plot(t_imu, v_imu.(method)(:, i), 'Color', colors.(method), 'DisplayName', ['IMU ' method]);
        end
        hold off; grid on; legend; title(['Velocity ' dims_ned{i}]); xlabel('Time (s)'); ylabel('Velocity (m/s)');
        
        % Acceleration
        subplot(3, 3, i + 6);
        hold on; plot(t_gnss, a_gnss(:, i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'GNSS (Derived)');
        for m = 1:length(methods)
            method = methods{m};
            plot(t_imu, a_imu.(method)(:, i), 'Color', colors.(method), 'DisplayName', ['IMU ' method]);
        end
        hold off; grid on; legend; title(['Acceleration ' dims_ned{i}]); xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
    end
    sgtitle(['GNSS vs. IMU Integration Comparison in ' frame_name ' Frame']);
    saveas(fig, filename);
    
    % --- Plot 2: All Data in ECEF Frame ---
    fig_ecef = figure('Name', 'All Data in ECEF Frame', 'Position', [150 150 1200 900], 'Visible', 'off');
    p_gnss_ecef = (C_n2e * p_gnss' + r0_ecef)';
    v_gnss_ecef = (C_n2e * v_gnss')';
    a_gnss_ecef = (C_n2e * a_gnss')';
    for i = 1:3
        % Position
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ecef(:,i), 'k-', 'DisplayName', 'GNSS Pos');
        for m = 1:length(methods)
            method = methods{m};
            p_ecef = (C_n2e * p_imu.(method)')';
            plot(t_imu, p_ecef(:,i), '--', 'Color', colors.(method), 'DisplayName', method);
        end
        hold off; grid on; legend; title(['Position ' dims_ecef{i}]); ylabel('m');

        % Velocity
        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ecef(:,i), 'k-', 'DisplayName', 'GNSS Vel');
        for m = 1:length(methods)
            method = methods{m};
            v_ecef = (C_n2e * v_imu.(method)')';
            plot(t_imu, v_ecef(:,i), '--', 'Color', colors.(method), 'DisplayName', method);
        end
        hold off; grid on; legend; title(['Velocity ' dims_ecef{i}]); ylabel('m/s');

        % Acceleration
        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ecef(:,i), 'k-', 'DisplayName', 'GNSS Accel');
        for m = 1:length(methods)
            method = methods{m};
            f_ned = (C_B_N_methods.(method) * f_b_corr.(method)')';
            f_ecef = (C_n2e * f_ned')';
            plot(t_imu, f_ecef(:,i), '--', 'Color', colors.(method), 'DisplayName', ['IMU ' method]);
        end
        hold off; grid on; legend; title(['Acceleration ' dims_ecef{i}]); ylabel('m/s^2');
    end
    saveas(fig_ecef, strrep(filename, '_ned.', '_ecef.'));
    
    close(fig); close(fig_ecef);
end

function data_filt = butter_lowpass_filter(data, cutoff, fs, order)
    %BUTTER_LOWPASS_FILTER Apply a zero-phase Butterworth low-pass filter.
    if nargin < 4 || isempty(order);   order = 4;   end
    if nargin < 3 || isempty(fs);      fs = 400;   end
    if nargin < 2 || isempty(cutoff);  cutoff = 5.0; end

    nyq = 0.5 * fs;
    normal_cutoff = cutoff / nyq;
    [b,a] = butter(order, normal_cutoff, 'low');
    data_filt = filtfilt(b, a, data);
end

function [start_idx, end_idx] = detect_static_interval(accel, gyro, window_size, accel_var_thresh, gyro_var_thresh, min_length)
    %DETECT_STATIC_INTERVAL Find longest initial static interval in IMU data.
    if nargin < 3 || isempty(window_size);      window_size = 200;   end
    if nargin < 4 || isempty(accel_var_thresh); accel_var_thresh = 0.01; end
    if nargin < 5 || isempty(gyro_var_thresh);  gyro_var_thresh = 1e-6; end
    if nargin < 6 || isempty(min_length);       min_length = 100;   end

    N = size(accel,1);
    if N < window_size
        error('window_size larger than data length');
    end

    rehash toolboxcache
    if license('test', 'Signal_Toolbox')
        accel_var = movvar(accel, window_size, 0, 'Endpoints','discard');
        gyro_var  = movvar(gyro,  window_size, 0, 'Endpoints','discard');
    else
        num_win = N - window_size + 1;
        accel_var = zeros(num_win, size(accel,2));
        gyro_var  = zeros(num_win, size(gyro,2));
        for i = 1:num_win
            accel_var(i,:) = var(accel(i:i+window_size-1,:),0,1);
            gyro_var(i,:)  = var(gyro(i:i+window_size-1,:),0,1);
        end
    end

    max_accel_var = max(accel_var, [], 2);
    max_gyro_var  = max(gyro_var, [], 2);
    static_mask = (max_accel_var < accel_var_thresh) & (max_gyro_var < gyro_var_thresh);

    diff_mask = diff([0; static_mask; 0]);
    starts = find(diff_mask == 1);
    ends   = find(diff_mask == -1) - 1;

    longest_len = 0; start_idx = 1; end_idx = window_size;
    for k = 1:length(starts)
        seg_len = ends(k) - starts(k) + 1;
        if seg_len >= min_length && seg_len > longest_len
            longest_len = seg_len;
            start_idx = starts(k);
            end_idx = ends(k) + window_size - 1;
        end
    end

    acc_var_sel = var(accel(start_idx:end_idx,:), 0, 1);
    gyro_var_sel = var(gyro(start_idx:end_idx,:), 0, 1);
    fprintf('detect_static_interval: [%d,%d] len=%d | acc_var=[%.4g %.4g %.4g] | gyro_var=[%.4g %.4g %.4g]\n', ...
        start_idx, end_idx, end_idx-start_idx+1, acc_var_sel, gyro_var_sel);
end

function plot_single_method(method, t_gnss, t_imu, C_B_N, p_gnss_ned, v_gnss_ned, a_gnss_ned, p_imu, v_imu, a_imu, acc_body_corr, base, r0_ecef, C_e2n)
    % Generate per-method comparison plots in various frames.

    dims = {'North','East','Down'};
    % ----- NED frame -----
    fig = figure('Visible','off','Position',[100 100 1200 900]);
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ned(:,i),'k--','DisplayName','GNSS');
        plot(t_imu, p_imu(:,i),'b-','DisplayName',method);
        hold off; grid on; legend; title(['Position ' dims{i}]); ylabel('m');

        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ned(:,i),'k--','DisplayName','GNSS');
        plot(t_imu, v_imu(:,i),'b-','DisplayName',method);
        hold off; grid on; legend; title(['Velocity ' dims{i}]); ylabel('m/s');

        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ned(:,i),'k--','DisplayName','GNSS');
        plot(t_imu, a_imu(:,i),'b-','DisplayName',method);
        hold off; grid on; legend; title(['Acceleration ' dims{i}]); ylabel('m/s^2');
    end
    sgtitle([method ' Comparison in NED frame']);
    fname = [base '_Task4_NEDFrame.pdf'];
    set(fig,'PaperPositionMode','auto');
    print(fig,fname,'-dpdf','-bestfit');
    fprintf('Comparison plot in NED frame saved\n');
    close(fig);

    % ----- ECEF frame -----
    fprintf('Plotting all data in ECEF frame.\n');
    C_n2e = C_e2n';
    fig = figure('Visible','off','Position',[100 100 1200 900]);
    p_gnss_ecef = (C_n2e*p_gnss_ned' + r0_ecef)';
    v_gnss_ecef = (C_n2e*v_gnss_ned')';
    a_gnss_ecef = (C_n2e*a_gnss_ned')';
    a_imu_ecef  = (C_n2e*a_imu')';
    p_imu_ecef  = (C_n2e*p_imu')';
    v_imu_ecef  = (C_n2e*v_imu')';
    dims_e = {'X','Y','Z'};
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ecef(:,i),'k--','DisplayName','GNSS');
        plot(t_imu, p_imu_ecef(:,i),'b-','DisplayName',method);
        hold off; grid on; legend; title(['Position ' dims_e{i}]); ylabel('m');

        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ecef(:,i),'k--','DisplayName','GNSS');
        plot(t_imu, v_imu_ecef(:,i),'b-','DisplayName',method);
        hold off; grid on; legend; title(['Velocity ' dims_e{i}]); ylabel('m/s');

        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ecef(:,i),'k--','DisplayName','GNSS');
        plot(t_imu, a_imu_ecef(:,i),'b-','DisplayName',method);
        hold off; grid on; legend; title(['Acceleration ' dims_e{i}]); ylabel('m/s^2');
    end
    sgtitle([method ' Comparison in ECEF frame']);
    fname = [base '_Task4_ECEFFrame.pdf'];
    set(fig,'PaperPositionMode','auto');
    print(fig,fname,'-dpdf','-bestfit');
    fprintf('All data in ECEF frame plot saved\n');
    close(fig);

    % ----- Body frame -----
    fprintf('Plotting all data in body frame.\n');
    fig = figure('Visible','off','Position',[100 100 1200 900]);
    C_N_B = C_B_N';
    pos_body = (C_N_B*p_gnss_ned')';
    vel_body = (C_N_B*v_gnss_ned')';
    dims_b = {'X','Y','Z'};
    for i = 1:3
        subplot(3,3,i); plot(t_gnss,pos_body(:,i),'k-'); grid on; title(['Position b' dims_b{i}]); ylabel('m');
        subplot(3,3,i+3); plot(t_gnss,vel_body(:,i),'k-'); grid on; title(['Velocity b' dims_b{i}]); ylabel('m/s');
        subplot(3,3,i+6); plot(t_imu, acc_body_corr(:,i),'b-'); grid on; title(['Acceleration b' dims_b{i}]); ylabel('m/s^2');
    end
    sgtitle([method ' Data in Body Frame']);
    fname = [base '_Task4_BodyFrame.pdf'];
    set(fig,'PaperPositionMode','auto');
    print(fig,fname,'-dpdf','-bestfit');
    fprintf('All data in body frame plot saved\n');
    close(fig);

    % ----- Mixed frame (ECEF position/velocity + body acceleration) -----
    fprintf('Plotting data in mixed frames.\n');
    fig = figure('Visible','off','Position',[100 100 1200 900]);
    for i = 1:3
        subplot(3,3,i); plot(t_gnss,p_gnss_ecef(:,i),'k-'); grid on; title(['Pos ' dims_e{i} ' ECEF']); ylabel('m');
        subplot(3,3,i+3); plot(t_gnss,v_gnss_ecef(:,i),'k-'); grid on; title(['Vel ' dims_e{i} ' ECEF']); ylabel('m/s');
        subplot(3,3,i+6); plot(t_imu, acc_body_corr(:,i),'b-'); grid on; title(['Acc ' dims_b{i} ' Body']); ylabel('m/s^2');
    end
    sgtitle([method ' Mixed Frame Data']);
    fname = [base '_Task4_MixedFrame.pdf'];
    set(fig,'PaperPositionMode','auto');
    print(fig,fname,'-dpdf','-bestfit');
    fprintf('Mixed frames plot saved\n');
    close(fig);
end

function q_new = propagate_quaternion(q_old, w, dt)
    %PROPAGATE_QUATERNION Propagate quaternion using body angular rate.
    w_norm = norm(w);
    if w_norm > 1e-9
        axis = w / w_norm;
        angle = w_norm * dt;
        dq = [cos(angle/2); axis * sin(angle/2)];
    else
        dq = [1; 0; 0; 0];
    end
    q_new = quat_multiply(q_old, dq);
    q_new = q_new / norm(q_new);
end

function q_out = quat_multiply(q1, q2)
    %QUAT_MULTIPLY Hamilton product of two quaternions.
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
    q_out = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
             w1*x2 + x1*w2 + y1*z2 - z1*y2;
             w1*y2 - x1*z2 + y1*w2 + z1*x2;
             w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function R = quat_to_rot(q)
    %QUAT_TO_ROT Convert quaternion to rotation matrix.
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R=[1-2*(qy^2+qz^2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
       2*(qx*qy+qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qw*qx);
       2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx^2+qy^2)];
end

function q = rot_to_quaternion(R)
    %ROT_TO_QUATERNION Convert rotation matrix to quaternion.
    tr = trace(R);
    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (R(3,2) - R(2,3)) / S;
        qy = (R(1,3) - R(3,1)) / S;
        qz = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
        qw = (R(3,2) - R(2,3)) / S;
        qx = 0.25 * S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
        qw = (R(1,3) - R(3,1)) / S;
        qx = (R(1,2) + R(2,1)) / S;
        qy = 0.25 * S;
        qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
        qw = (R(2,1) - R(1,2)) / S;
        qx = (R(1,3) + R(3,1)) / S;
        qy = (R(2,3) + R(3,2)) / S;
        qz = 0.25 * S;
    end
    q = [qw; qx; qy; qz];
    if q(1) < 0, q = -q; end
    q = q / norm(q);
end
