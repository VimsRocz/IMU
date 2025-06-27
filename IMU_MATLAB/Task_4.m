function Task_4(imu_file, gnss_file, method)
%TASK_4 GNSS and IMU data integration and comparison
%   Task_4(IMUFILE, GNSSFILE, METHOD) runs the GNSS/IMU integration
%   using the attitude estimates from Task 3. METHOD is unused but kept
%   for backwards compatibility with older scripts.
%   Requires that `Task_3` has already saved `results/task3_results.mat`.

if nargin < 1 || isempty(imu_file)
    error('IMU file not specified');
end
if nargin < 2 || isempty(gnss_file)
    error('GNSS file not specified');
end
if nargin < 3
    method = ''; %#ok<NASGU>
end

if ~exist('results','dir')
    mkdir('results');
end
results_dir = 'results';
[~, imu_name, ~] = fileparts(imu_file);
[~, gnss_name, ~] = fileparts(gnss_file);
tag = [imu_name '_' gnss_name];

% Load rotation matrices produced by Task 3
results_file = fullfile(results_dir, 'task3_results.mat');
if ~isfile(results_file)
    error('Task 3 results not found: %s', results_file);
end
data = load(results_file);
if ~isfield(data, 'task3_results')
    error('Variable ''task3_results'' missing from %s', results_file);
end
task3_results = data.task3_results;

fprintf('\nTASK 4: GNSS and IMU Data Integration and Comparison\n');

%% ========================================================================
% Subtask 4.1: Access Rotation Matrices from Task 3
% =========================================================================
fprintf('\nSubtask 4.1: Accessing rotation matrices from Task 3.\n');
methods = fieldnames(task3_results); % e.g., 'TRIAD', 'Davenport', 'SVD'
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
gnss_path = get_data_file(gnss_file);
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
[lat_deg_ref, lon_deg_ref, ~] = ecef_to_geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
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


%% ========================================================================
% Subtask 4.7: Convert GNSS Data to NED Frame
% =========================================================================
fprintf('\nSubtask 4.7: Converting GNSS data to NED frame.\n');
gnss_pos_ned = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';
fprintf('-> GNSS data transformed to NED frame.\n');


%% ========================================================================
% Subtask 4.8: Estimate GNSS Acceleration in NED
% =========================================================================
fprintf('\nSubtask 4.8: Estimating GNSS acceleration in NED.\n');
dt_gnss = diff(gnss_time);
% Prepend a zero row to maintain size, as diff reduces length by 1
gnss_accel_ned = [zeros(1,3); diff(gnss_vel_ned) ./ dt_gnss];
fprintf('-> GNSS acceleration estimated in NED frame.\n');


%% ========================================================================
% Subtask 4.9: Load IMU Data and Correct for Bias for Each Method
% =========================================================================
fprintf('\nSubtask 4.9: Loading IMU data and correcting for bias for each method.\n');
imu_path = get_data_file(imu_file);
imu_raw_data = readmatrix(imu_path);
dt_imu = mean(diff(imu_raw_data(1:100,2)));
if dt_imu <= 0 || isnan(dt_imu), dt_imu = 1/400; end
imu_time = (0:size(imu_raw_data,1)-1)' * dt_imu + gnss_time(1);

acc_body_raw = imu_raw_data(:, 6:8) / dt_imu;
acc_body_filt = butter_lowpass_filter(acc_body_raw, 5.0, 1/dt_imu);
gyro_body_filt = butter_lowpass_filter(imu_raw_data(:, 3:5) / dt_imu, 5.0, 1/dt_imu);

[start_idx, end_idx] = detect_static_interval(acc_body_filt, gyro_body_filt);
static_acc = mean(acc_body_filt(start_idx:end_idx, :), 1);
g_NED = [0; 0; 9.81];

acc_body_corrected = struct();
for i = 1:length(methods)
    method = methods{i};
    C_B_N = C_B_N_methods.(method);
    C_N_B = C_B_N';

    g_body_expected = C_N_B * g_NED;
    acc_bias = static_acc' + g_body_expected;
    scale = 9.81 / norm(static_acc' - acc_bias);
    acc_body_corrected.(method) = (acc_body_filt - acc_bias') * scale;
    fprintf('Method %s: Accel bias=[%.4f, %.4f, %.4f], Scale=%.4f\n', method, acc_bias, scale);
end
fprintf('-> IMU data corrected for bias and scale for each method.\n');


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
    
    pos = zeros(size(imu_time,1), 3);
    vel = zeros(size(imu_time,1), 3);
    acc = zeros(size(imu_time,1), 3);
    
    % Initialize with first GNSS measurement
    vel(1,:) = gnss_vel_ned(1,:);
    pos(1,:) = gnss_pos_ned(1,:);
    
    for k = 2:length(imu_time)
        f_ned = C_B_N * acc_body_corrected.(method)(k,:)';
        a_ned = f_ned + g_NED;
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

plot_comparison_in_frame( ...
    'NED', gnss_time, imu_time, methods, C_B_N_methods, ...
    gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, ...
    pos_integ, vel_integ, acc_integ, acc_body_corrected, ...
    fullfile(results_dir, [tag '_comparison_ned.pdf']), ref_r0, C_ECEF_to_NED ...
);
fprintf('-> All data plots saved.\n');

% Save GNSS positions for use by Task 5
task4_file = fullfile(results_dir, 'task4_results.mat');
if isfile(task4_file)
    save(task4_file, 'gnss_pos_ned', '-append');
else
    save(task4_file, 'gnss_pos_ned');
end
% Task 5 loads these positions when initialising the Kalman filter

end % End of main function: run_task4


%% ========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function C = compute_C_ECEF_to_NED(lat_rad, lon_rad)
    % Compute rotation matrix from ECEF to NED frame.
    s_lat = sin(lat_rad); c_lat = cos(lat_rad);
    s_lon = sin(lon_rad); c_lon = cos(lon_rad);

    C = [-s_lat * c_lon, -s_lat * s_lon,  c_lat;
         -s_lon,          c_lon,         0;
         -c_lat * c_lon, -c_lat * s_lon, -s_lat];
end

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
    fig_ecef = figure('Name', 'All Data in ECEF Frame', 'Position', [150 150 1200 600], 'Visible', 'off');
    p_gnss_ecef = (C_n2e * p_gnss' + r0_ecef)';
    v_gnss_ecef = (C_n2e * v_gnss')';
    a_gnss_ecef = (C_n2e * a_gnss')';
    for i = 1:3
        subplot(3,1,i);
        hold on;
        plot(t_gnss, p_gnss_ecef(:,i), 'k-', 'DisplayName', 'GNSS Pos');
        plot(t_gnss, v_gnss_ecef(:,i), 'b-', 'DisplayName', 'GNSS Vel');
        plot(t_gnss, a_gnss_ecef(:,i), 'r-', 'DisplayName', 'GNSS Accel');
        for m = 1:length(methods)
            method = methods{m};
            f_ned = (C_B_N_methods.(method) * f_b_corr.(method)')';
            f_ecef = (C_n2e * f_ned')';
            plot(t_imu, f_ecef(:,i), '--', 'Color', colors.(method), 'DisplayName', ['IMU Force ' method]);
        end
        hold off; grid on; legend; title(['ECEF Frame - Axis ' dims_ecef{i}]); xlabel('Time (s)');
    end
    saveas(fig_ecef, strrep(filename, '_ned.', '_ecef.'));
    
    close(fig); close(fig_ecef);
end
