function result = Task_4(imu_path, gnss_path, method)
%TASK_4 GNSS and IMU data integration and comparison
%   Task_4(IMU_PATH, GNSS_PATH, METHOD) runs the GNSS/IMU integration
%   using the attitude estimates from Task 3. METHOD is unused but kept
%   for backwards compatibility with older scripts.
%   Requires that `Task_3` has already saved a dataset-specific
%   results file under `results/` such as
%   `Task3_results_IMU_X001_GNSS_X001.mat`.
%   Task 4 expects the bias estimates from Task 2. These must be stored
%   with the variable names ``accel_bias`` and ``gyro_bias`` within
%   ``Task2_body_<tag>.mat``. Legacy fields like ``acc_bias`` are
%   supported but discouraged.
%
% Usage:
%   Task_4(imu_path, gnss_path, method)

paths = project_paths();
results_dir = paths.matlab_results;
addpath(fullfile(paths.root,'MATLAB','lib'));
addpath(fullfile(paths.root,'src','utils'));  % for detect_imu_time

% pull configuration from caller
try
    cfg = evalin('caller','cfg');
catch
    error('cfg not found in caller workspace');
end

if nargin < 1 || isempty(imu_path)
    error('IMU file not specified');
end
if nargin < 2 || isempty(gnss_path)
    error('GNSS file not specified');
end
if nargin < 3
    method = '';
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

[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);
imu_id  = erase(imu_base, '.dat');
gnss_id = erase(gnss_base, '.csv');
imu_name = imu_id;    % legacy variable names
gnss_name = gnss_id;
pair_tag = [imu_id '_' gnss_id];
if isempty(method)
    tag = pair_tag;
    method_tag = 'AllMethods';
else
    tag = [pair_tag '_' method];
    method_tag = method;
end
run_id = sprintf('%s_%s_%s', imu_id, gnss_id, method);

% Load accelerometer and gyroscope biases estimated in Task 2
task2_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', ...
    imu_id, gnss_id, method_tag));
assert(isfile(task2_file), 'Task 4: missing %s', task2_file);
S2 = load(task2_file);
if isfield(S2, 'body_data')
    bd = S2.body_data;
else
    error('body_data missing from %s', task2_file);
end
acc_bias = bd.accel_bias(:).';
gyro_bias = bd.gyro_bias(:).';

% Load rotation matrices produced by Task 3
cand1 = fullfile(results_dir, sprintf('Task3_results_%s_%s.mat', imu_id, gnss_id));
cand2 = fullfile(results_dir, sprintf('%s_%s_%s_task3_results.mat', imu_id, gnss_id, method));
if isfile(cand1)
    S3 = load(cand1);
elseif isfile(cand2)
    S3 = load(cand2);
else
    error('Task 4: Task 3 results missing. Tried:\n  %s\n  %s', cand1, cand2);
end
if ~isfield(S3, 'task3_results')
    error('Task 4: variable ''task3_results'' missing from Task 3 MAT file.');
end
task3_results = S3.task3_results;

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
all_methods = task3_results.methods; % e.g., 'TRIAD', 'Davenport', 'SVD'
if ~isempty(method)
    methods = {method};
else
    methods = all_methods;
end
C_B_N_methods = struct();
for i = 1:length(methods)
    method_name = methods{i};
    C_B_N_methods.(method_name) = task3_results.Rbn.(method_name);
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
gnss_pos_ecef = gnss_data{:, pos_cols};
vx = gnss_data.VX_ECEF_mps;
vy = gnss_data.VY_ECEF_mps;
vz = gnss_data.VZ_ECEF_mps;
gnss_vel_ecef = [vx vy vz];
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
% Subtask 4.6: Computing ECEF to NED rotation matrix.
% =========================================================================
fprintf('\nSubtask 4.6: Computing ECEF to NED rotation matrix.\n');
lat_rad = ref_lat;
lon_rad = ref_lon;
phi = lat_rad;           % latitude  (rad)
lam = lon_rad;           % longitude (rad)

% ECEF -> NED (NED rows expressed in ECEF basis)
R_ecef_to_ned = [ -sin(phi)*cos(lam), -sin(phi)*sin(lam),  cos(phi);
                  -sin(lam),           cos(lam),           0;
                  -cos(phi)*cos(lam), -cos(phi)*sin(lam), -sin(phi) ];
R_ned_to_ecef = R_ecef_to_ned.';  % orthonormal => inverse is transpose

% Orthonormality checks (use matrix, not vectorized I(:))
I1 = R_ecef_to_ned * R_ned_to_ecef;
I2 = R_ned_to_ecef * R_ecef_to_ned;
err1 = norm(I1 - eye(3), 'fro');
err2 = norm(I2 - eye(3), 'fro');
assert(err1 < 1e-10 && err2 < 1e-10, ...
    sprintf('R_ecef_to_ned not orthonormal (errs: %.3e, %.3e)', err1, err2));

C_ECEF_to_NED = R_ecef_to_ned;
C_NED_to_ECEF = R_ned_to_ecef;
disp('-> ECEF to NED rotation matrix computed.');
disp('-> NED to ECEF rotation matrix computed.');


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

% Build GNSS time vector
Tg = readtable(gnss_path);
if any(strcmp(Tg.Properties.VariableNames,'Posix_Time'))
    t_gnss = zero_base_time(Tg.Posix_Time(:));
else
    t_gnss = (0:height(Tg)-1)';   % assume 1 Hz if missing
end


%% ========================================================================
% Subtask 4.8: Estimate GNSS Acceleration in NED
% =========================================================================
fprintf('\nSubtask 4.8: Estimating GNSS acceleration in NED.\n');
dt_gnss = diff(t_gnss(:));
% Prepend a zero row to maintain size, as diff reduces length by 1
gnss_accel_ned = [zeros(1,3); bsxfun(@rdivide, diff(gnss_vel_ned), dt_gnss)];
fprintf('-> GNSS acceleration estimated in NED frame.\n');
fprintf('   GNSS accel RMS = %.4f m/s^2\n', rms(gnss_accel_ned(:)) );


%% ========================================================================
% Subtask 4.9: Load IMU Data and Correct for Bias for Each Method
% =========================================================================
fprintf('\nSubtask 4.9: Loading IMU data and correcting for bias for each method.\n');
Ti = readmatrix(imu_path);
t_imu = detect_imu_time(Ti, 0.0025);   % unwrap rollover or fallback to 400 Hz
imu_raw_data = Ti;
dt_imu = median(diff(t_imu));
if dt_imu <= 0 || isnan(dt_imu), dt_imu = 0.0025; end
imu_time = t_imu;

acc_body_raw = imu_raw_data(:, 6:8) / dt_imu;

% === Correct IMU raw acc and transform to NED linear acceleration ===
imu_acc_corr_body = acc_body_raw - acc_bias;                 % bias removal
C_bn = C_B_N_methods.TRIAD;                                  % body->NED
imu_acc_ned = (C_bn * imu_acc_corr_body.').';               % Nx3

%% ========================================================================
% Subtask 4.10: Setting IMU parameters and gravity vector.
% =========================================================================
% Try to load gravity from Task 1; otherwise compute from WGS-84.
p = project_paths();
results_dir = p.matlab_results;

[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);
imu_id  = erase(imu_base, '.dat');
gnss_id = erase(gnss_base, '.csv');

t1_file = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method));
g_loaded = false;
g_NED = [NaN;NaN;NaN];

if isfile(t1_file)
    S1 = load(t1_file);
    cand_names = {'g_ned','gravity_ned','gravity_vec_ned','gravity_vector','gNED'};
    for k = 1:numel(cand_names)
        if isfield(S1, cand_names{k})
            g_NED = S1.(cand_names{k})(:);
            g_loaded = all(isfinite(g_NED)) && numel(g_NED)==3;
            if g_loaded, break; end
        end
    end
end

if ~g_loaded
    % Compute normal gravity at current latitude / height (WGS-84)
    h_m = 0;   % default height if we can\u2019t read it
    if exist('GNSS','var') && istable(GNSS) && any(strcmpi(GNSS.Properties.VariableNames,'Height_deg'))
        h_m = GNSS.Height_deg(1);  % file uses "Height_deg" but values are meters
    elseif exist('gnss_data','var') && istable(gnss_data) && any(strcmpi(gnss_data.Properties.VariableNames,'Height_deg'))
        h_m = gnss_data.Height_deg(1);
    elseif exist('height_m','var')
        h_m = height_m;
    end
    g_mag = normal_gravity_wgs84(lat_rad, h_m);
    g_NED = [0; 0; g_mag];
    fprintf('Gravity (fallback): WGS-84 at lat=%.6f rad, h=%.1f m -> %.8f m/s^2 (NED +Z down)\n', ...
            lat_rad, h_m, g_mag);
else
    fprintf('Gravity loaded from Task 1: [%.8f %.8f %.8f] m/s^2 (NED)\n', g_NED);
end

% IMU sample interval dt expected earlier as "dt"
dt = dt_imu;
assert(exist('dt','var')==1, 'IMU dt not set before Subtask 4.10');

gravity_mag = norm(g_NED);
gNED = [0 0 gravity_mag];
imu_linacc_ned = imu_acc_ned - gNED;                        % remove gravity

gnss_acc_ned = gnss_accel_ned;                              % from GNSS

% Interpolate GNSS NED signals onto IMU time grid (so lengths match)
gnss_pos_ned_imuT = interp1(t_gnss, gnss_pos_ned, t_imu, 'linear', 'extrap');
gnss_vel_ned_imuT = interp1(t_gnss, gnss_vel_ned, t_imu, 'linear', 'extrap');
gnss_acc_ned_imuT = interp1(t_gnss, gnss_acc_ned, t_imu, 'linear', 'extrap');

acc_body_filt = butter_lowpass_filter(acc_body_raw, 5.0, 1/dt_imu);
gyro_body_filt = butter_lowpass_filter(imu_raw_data(:, 3:5) / dt_imu, 5.0, 1/dt_imu);
acc_rms  = sqrt(mean(acc_body_raw(:).^2));
gyro_rms = sqrt(mean((imu_raw_data(:,3:5)/dt_imu).^2,'all'));
fprintf('   Acc raw RMS=%.4f, Gyro raw RMS=%.6f\n', acc_rms, gyro_rms);

[start_idx, end_idx] = detect_static_interval(acc_body_filt, gyro_body_filt);
static_acc  = mean(acc_body_filt(start_idx:end_idx, :), 1);
static_gyro = mean(gyro_body_filt(start_idx:end_idx, :), 1);
acc_var = var(acc_body_filt(start_idx:end_idx, :), 0, 1);
gyro_var = var(gyro_body_filt(start_idx:end_idx, :), 0, 1);
fprintf('Static interval [%d, %d] (len=%d)\n', start_idx, end_idx, end_idx-start_idx+1);
fprintf('Static acc mean  =[%.4f %.4f %.4f]\n', static_acc);
fprintf('Static gyro mean =[%.6f %.6f %.6f]\n', static_gyro);
fprintf('Static acc var   =[%.4g %.4g %.4g]\n', acc_var);
fprintf('Static gyro var  =[%.4g %.4g %.4g]\n', gyro_var);

% --- accelerometer scale estimate from static segment ---
body_static_acc = acc_body_filt(start_idx:end_idx, :);
g = 9.79424753;  % close enough for scale estimate
mag = mean(vecnorm(body_static_acc,2,2),'omitnan');
scale_est = g / mag;
if scale_est < 0.8 || scale_est > 1.2
    warning('Task 4: accel scale %.4f out of [0.8,1.2]; falling back to 1.0', scale_est);
    scale_est = 1.0;
end
accel_scale = scale_est;
fprintf('Task 4: estimated accelerometer scale factor = %.4f\n', accel_scale);

imu_linacc_ned_scaled = accel_scale * imu_linacc_ned;

bd.accel_scale = accel_scale;
body_data = bd;
save(task2_file, 'body_data', '-v7.3');

omega_E = constants.EARTH_RATE;                     % rad/s
omega_ie_NED = omega_E * [cos(ref_lat); 0; -sin(ref_lat)];

% Correct IMU measurements separately for each Wahba method
acc_body_corrected  = struct();
gyro_body_corrected = struct();
acc_biases = struct();
gyro_biases = struct();
scale_factors = struct();

for i = 1:length(methods)
    method = methods{i};
    acc_body_corrected.(method)  = (acc_body_filt - acc_bias) * accel_scale;
    gyro_body_corrected.(method) = gyro_body_filt - gyro_bias;
    acc_biases.(method)  = acc_bias';
    gyro_biases.(method) = gyro_bias';
    scale_factors.(method) = accel_scale;

    fprintf('Method %s: Accelerometer bias: [%10.8f %10.8f %10.8f] (|b|=%.6f m/s^2)\n', ...
            method, acc_bias', norm(acc_bias));
    fprintf('Method %s: Gyroscope bias: [% .8e % .8e % .8e]\n', method, gyro_bias');
    fprintf('Method %s: Accelerometer scale factor: %.4f\n', method, accel_scale);
end
fprintf('-> IMU data corrected for bias and scale for each method.\n');
for i = 1:length(methods)
    m = methods{i};
    fprintf('Task 4: applied accelerometer scale factor = %.4f, bias = [% .4f % .4f % .4f]\n', ...
        scale_factors.(m), acc_biases.(m));
end
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
    
    % Initialize with first GNSS measurement (aligned to IMU time)
    vel(1,:) = gnss_vel_ned_imuT(1,:);
    pos(1,:) = gnss_pos_ned_imuT(1,:);
    
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

% --- Align rates for validation on the GNSS timeline ---
method_first = methods{1};
t_g = t_gnss(:); t_i = t_imu(:);

% sanitize time vectors to satisfy interp1 requirements
[t_i_u, pos_u] = ensure_unique_increasing('IMU time', t_i, pos_integ.(method_first));
[~,       vel_u] = ensure_unique_increasing('IMU time', t_i, vel_integ.(method_first));
[~,       acc_u] = ensure_unique_increasing('IMU time', t_i, acc_integ.(method_first));
[t_g_u, gnss_pos_u] = ensure_unique_increasing('GNSS time', t_g, gnss_pos_ned);
[~,      gnss_vel_u] = ensure_unique_increasing('GNSS time', t_g, gnss_vel_ned);
[~,      gnss_acc_u] = ensure_unique_increasing('GNSS time', t_g, gnss_acc_ned);

fprintf('[Task4] unique t_i: %d -> %d (dups dropped=%d)\n', ...
    numel(t_i), numel(t_i_u), numel(t_i)-numel(t_i_u));
fprintf('[Task4] unique t_g: %d -> %d (dups dropped=%d)\n', ...
    numel(t_g), numel(t_g_u), numel(t_g)-numel(t_g_u));

imu_pos_g  = interp1(t_i_u, pos_u, t_g_u, 'linear','extrap');
imu_vel_g  = interp1(t_i_u, vel_u, t_g_u, 'linear','extrap');
imu_acc_g  = interp1(t_i_u, acc_u, t_g_u, 'linear','extrap');
valid = all(isfinite(gnss_acc_u),2) & all(isfinite(imu_acc_g),2);
t_v = t_g_u(valid);
pos_v = imu_pos_g(valid,:); vel_v = imu_vel_g(valid,:); acc_v = imu_acc_g(valid,:);

plot_state_grid(t_v, pos_v, vel_v, acc_v, 'NED', ...
    'visible','on', 'save_dir', results_dir, 'run_id', run_id);

% -------------------------------------------------------------------------
% Generate comparison plots for all methods in NED, ECEF, BODY and Mixed
% frames using the helper ``plot_frame_comparison``.
% -------------------------------------------------------------------------
t = t_imu; % Common time vector at IMU rate
pos_ned_GNSS = gnss_pos_ned_imuT';
pos_ecef_GNSS = C_NED_to_ECEF * pos_ned_GNSS + ref_r0;

% NED/ECEF/BODY positions for each method
pos_ned = struct();
pos_ecef = struct();
pos_body = struct();
for i = 1:length(methods)
    m = methods{i};
    pos_ned.(m)  = pos_integ.(m)';
    pos_ecef.(m) = C_NED_to_ECEF * pos_ned.(m) + ref_r0;
end

% Body-frame positions (use TRIAD attitude if available, otherwise first method)
if isfield(C_B_N_methods, 'TRIAD')
    C_N_B_ref = C_B_N_methods.TRIAD';
else
    C_N_B_ref = C_B_N_methods.(methods{1})';
end
pos_body_GNSS = C_N_B_ref * pos_ned_GNSS;
for i = 1:length(methods)
    m = methods{i};
    pos_body.(m) = C_N_B_ref * pos_ned.(m);
end

prefix = fullfile(results_dir, sprintf('%s_task4', run_id));

% Assemble datasets in a fixed method order for plotting
method_order = {'TRIAD','Davenport','SVD'};
data_sets = {pos_ned_GNSS};
data_sets_ecef = {pos_ecef_GNSS};
data_sets_body = {pos_body_GNSS};
labels = {'GNSS'};
for i = 1:length(method_order)
    m = method_order{i};
    if isfield(pos_ned, m)
        data_sets{end+1} = pos_ned.(m);
        data_sets_ecef{end+1} = pos_ecef.(m);
        data_sets_body{end+1} = pos_body.(m);
        labels{end+1} = ['IMU-' m];
    end
end

plot_frame_comparison(t, data_sets, labels, 'NED',  prefix, cfg);
plot_frame_comparison(t, data_sets_ecef, labels, 'ECEF', prefix, cfg);
plot_frame_comparison(t, data_sets_body, labels, 'BODY', prefix, cfg);
plot_frame_comparison(t, data_sets, labels, 'Mixed', prefix, cfg);


%% ========================================================================
% Subtask 4.12b: Load truth ECEF trajectory if available
% =========================================================================
state_file = fullfile(fileparts(imu_path), sprintf('STATE_%s.txt', imu_name));
truth_pos_ecef = [];
truth_vel_ecef = [];
truth_time = [];
if isfile(state_file)
    try
        truth_data = read_state_file(state_file);
        % Preserve time as a column vector for downstream tasks
        truth_time = truth_data(:,2);
        truth_time = truth_time(:);
        truth_pos_ecef = truth_data(:,3:5);
        truth_vel_ecef = truth_data(:,6:8);
        fprintf('Loaded truth ECEF trajectory from %s\n', state_file);
    catch ME
        warning('Failed to load truth state file: %s', ME.message);
    end
else
    fprintf('Truth state file not found: %s\n', state_file);
end


%% ========================================================================
% Subtask 4.13: Validate and Plot Data
% =========================================================================
fprintf('\nSubtask 4.13: Validating and plotting data.\n');

for i = 1:length(methods)
    m = methods{i};
    plot_single_method(m, t_imu, t_imu, C_B_N_methods.(m), ...
        gnss_pos_ned_imuT, gnss_vel_ned_imuT, gnss_acc_ned_imuT, ...
        pos_integ.(m), vel_integ.(m), acc_integ.(m), ...
        acc_body_corrected.(m), run_id, ref_r0, C_ECEF_to_NED, cfg);
end
fprintf('-> All data plots saved for all methods.\n');

% Save GNSS positions for use by Task 5
task4_file = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
if isfile(task4_file)
    save(task4_file, 'gnss_pos_ned', 'acc_biases', 'gyro_biases', 'scale_factors', ...
        'truth_pos_ecef', 'truth_vel_ecef', 'truth_time', ...
        't_imu','t_gnss', 'gnss_pos_ned_imuT','gnss_vel_ned_imuT','gnss_acc_ned_imuT', '-append');
else
    save(task4_file, 'gnss_pos_ned', 'acc_biases', 'gyro_biases', 'scale_factors', ...
        'truth_pos_ecef', 'truth_vel_ecef', 'truth_time', ...
        't_imu','t_gnss', 'gnss_pos_ned_imuT','gnss_vel_ned_imuT','gnss_acc_ned_imuT');
end
fprintf('GNSS NED positions saved to %s\n', task4_file);

% Save method-specific results using helper
result_struct = struct('gnss_pos_ned', gnss_pos_ned, 'acc_biases', acc_biases, ...
                'gyro_biases', gyro_biases, 'scale_factors', scale_factors, ...
                'truth_pos_ecef', truth_pos_ecef, 'truth_vel_ecef', truth_vel_ecef, ...
                'truth_time', truth_time, 't_imu', t_imu, 't_gnss', t_gnss, ...
                'gnss_pos_ned_imuT', gnss_pos_ned_imuT, ...
                'gnss_vel_ned_imuT', gnss_vel_ned_imuT, ...
                'gnss_acc_ned_imuT', gnss_acc_ned_imuT);
save_task_results(result_struct, imu_name, gnss_name, method_tag, 4);
% Task 5 loads these positions when initialising the Kalman filter

% Return results structure and store in base workspace
result = struct('gnss_pos_ned', gnss_pos_ned, 'acc_biases', acc_biases, ...
                'gyro_biases', gyro_biases, 'scale_factors', scale_factors, ...
                'truth_pos_ecef', truth_pos_ecef, 'truth_vel_ecef', truth_vel_ecef, ...
                'truth_time', truth_time, 't_imu', t_imu, 't_gnss', t_gnss, ...
                'gnss_pos_ned_imuT', gnss_pos_ned_imuT, ...
                'gnss_vel_ned_imuT', gnss_vel_ned_imuT, ...
                'gnss_acc_ned_imuT', gnss_acc_ned_imuT);
assignin('base', 'task4_results', result);

end % End of main function: run_task4


%% ========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function plot_comparison_in_frame(frame_name, t_gnss, t_imu, methods, C_B_N_methods, p_gnss, v_gnss, a_gnss, p_imu, v_imu, a_imu, f_b_corr, filename, r0_ecef, C_e2n)
    %PLOT_COMPARISON_IN_FRAME Visualise GNSS and IMU data in multiple frames.
    %   PLOT_COMPARISON_IN_FRAME(FRAME_NAME, T_GNSS, T_IMU, METHODS, C_B_N_METHODS,
    %   P_GNSS, V_GNSS, A_GNSS, P_IMU, V_IMU, A_IMU, F_B_CORR, FILENAME, R0_ECEF,
    %   C_E2N) generates comparison plots for the given attitude
    %   determination METHODS.  GNSS data are provided in NED and the IMU
    %   derived position, velocity and acceleration are passed in the same
    %   frame.  Results are saved to FILENAME.  R0_ECEF and C_E2N are used to
    %   convert data to the ECEF frame for additional plots.
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
    %   DATA_FILT = BUTTER_LOWPASS_FILTER(DATA, CUTOFF, FS, ORDER) filters DATA
    %   along its first dimension using a low-pass Butterworth design.  CUTOFF
    %   is the cut-off frequency in Hz, FS the sample rate in Hz and ORDER the
    %   filter order.  Defaults are 5&nbsp;Hz, 400&nbsp;Hz and order 4.
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
    %   [I0, I1] = DETECT_STATIC_INTERVAL(ACCEL, GYRO, WINDOW_SIZE,
    %   ACCEL_VAR_THRESH, GYRO_VAR_THRESH, MIN_LENGTH) returns the start and end
    %   indices of the longest segment where the variance of ACCEL and GYRO are
    %   below the given thresholds.  Parameters mirror the Python helper
    %   ``detect_static_interval`` found in ``utils.py``.
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

function plot_single_method(method, t_gnss, t_imu, C_B_N, p_gnss_ned, v_gnss_ned, a_gnss_ned, p_imu, v_imu, a_imu, acc_body_corr, run_id, r0_ecef, C_e2n, cfg)
    %PLOT_SINGLE_METHOD Helper to produce per-method comparison figures.
    %   PLOT_SINGLE_METHOD(METHOD, T_GNSS, T_IMU, C_B_N, P_GNSS_NED, V_GNSS_NED,
    %   A_GNSS_NED, P_IMU, V_IMU, A_IMU, ACC_BODY_CORR, RUN_ID, R0_ECEF, C_E2N, CFG)
    %   generates plots comparing the IMU integration against GNSS in NED,
    %   ECEF and body frames. RUN_ID and CFG control output filenames and
    %   plotting policy.

    dims = {'North','East','Down'};
    gnss_col  = [0.8500 0.3250 0.0980];
    fused_col = [0 0.4470 0.7410];
    % ----- NED frame -----
    base = fullfile(cfg.paths.matlab_results, sprintf('%s_task4', run_id));
    fig = figure('Visible', ternary(cfg.plots.popup_figures,'on','off'), 'Position',[100 100 1200 900]);
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ned(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        plot(t_imu,  p_imu(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Position ' dims{i}]); ylabel('m'); xlabel('Time (s)');

        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ned(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        plot(t_imu,  v_imu(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Velocity ' dims{i}]); ylabel('m/s'); xlabel('Time (s)');

        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ned(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        plot(t_imu,  a_imu(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Acceleration ' dims{i}]); ylabel('m/s^2'); xlabel('Time (s)');
    end
    sgtitle([method ' Comparison in NED frame']);
    fname = [base '_NED_state'];
    set(fig,'PaperPositionMode','auto');
    if cfg.plots.save_pdf
        print(fig,[fname '.pdf'],'-dpdf','-bestfit');
    end
    if cfg.plots.save_png
        print(fig,[fname '.png'],'-dpng');
    end
    fprintf('Comparison plot in NED frame saved\n');
    close(fig);

    % ----- ECEF frame -----
    fprintf('Plotting all data in ECEF frame.\n');
    C_n2e = C_e2n';
    fig = figure('Visible', ternary(cfg.plots.popup_figures,'on','off'), 'Position',[100 100 1200 900]);
    p_gnss_ecef = (C_n2e*p_gnss_ned' + r0_ecef)';
    v_gnss_ecef = (C_n2e*v_gnss_ned')';
    a_gnss_ecef = (C_n2e*a_gnss_ned')';
    a_imu_ecef  = (C_n2e*a_imu')';
    p_imu_ecef  = (C_n2e*p_imu')';
    v_imu_ecef  = (C_n2e*v_imu')';
    dims_e = {'X','Y','Z'};
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ecef(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        plot(t_imu,  p_imu_ecef(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Position ' dims_e{i}]); ylabel('m'); xlabel('Time (s)');

        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ecef(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        plot(t_imu,  v_imu_ecef(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Velocity ' dims_e{i}]); ylabel('m/s'); xlabel('Time (s)');

        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ecef(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        plot(t_imu,  a_imu_ecef(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Acceleration ' dims_e{i}]); ylabel('m/s^2'); xlabel('Time (s)');
    end
    sgtitle([method ' Comparison in ECEF frame']);
    fname = [base '_ECEF_state'];
    set(fig,'PaperPositionMode','auto');
    if cfg.plots.save_pdf
        print(fig,[fname '.pdf'],'-dpdf','-bestfit');
    end
    if cfg.plots.save_png
        print(fig,[fname '.png'],'-dpng');
    end
    fprintf('All data in ECEF frame plot saved\n');
    close(fig);

    % ----- Body frame -----
    fprintf('Plotting all data in body frame.\n');
    fig = figure('Visible', ternary(cfg.plots.popup_figures,'on','off'), 'Position',[100 100 1200 900]);
    C_N_B = C_B_N';
    pos_body = (C_N_B*p_gnss_ned')';
    vel_body = (C_N_B*v_gnss_ned')';
    dims_b = {'X','Y','Z'};
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss,pos_body(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        hold off; grid on; legend; title(['Position b' dims_b{i}]); ylabel('m'); xlabel('Time (s)');

        subplot(3,3,i+3); hold on;
        plot(t_gnss,vel_body(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        hold off; grid on; legend; title(['Velocity b' dims_b{i}]); ylabel('m/s'); xlabel('Time (s)');

        subplot(3,3,i+6); hold on;
        plot(t_imu, acc_body_corr(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Acceleration b' dims_b{i}]); ylabel('m/s^2'); xlabel('Time (s)');
    end
    sgtitle([method ' Data in Body Frame']);
    fname = [base '_BODY_state'];
    set(fig,'PaperPositionMode','auto');
    if cfg.plots.save_pdf
        print(fig,[fname '.pdf'],'-dpdf','-bestfit');
    end
    if cfg.plots.save_png
        print(fig,[fname '.png'],'-dpng');
    end
    fprintf('All data in body frame plot saved\n');
    close(fig);

    % ----- Mixed frame (ECEF position/velocity + body acceleration) -----
    fprintf('Plotting data in mixed frames.\n');
    fig = figure('Visible','off','Position',[100 100 1200 900]);
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss,p_gnss_ecef(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        hold off; grid on; legend; title(['Pos ' dims_e{i} ' ECEF']); ylabel('m'); xlabel('Time (s)');

        subplot(3,3,i+3); hold on;
        plot(t_gnss,v_gnss_ecef(:,i),'--','Color',gnss_col,'DisplayName','GNSS (integrated)');
        hold off; grid on; legend; title(['Vel ' dims_e{i} ' ECEF']); ylabel('m/s'); xlabel('Time (s)');

        subplot(3,3,i+6); hold on;
        plot(t_imu, acc_body_corr(:,i),'-','Color',fused_col,'DisplayName','Fused');
        hold off; grid on; legend; title(['Acc ' dims_b{i} ' Body']); ylabel('m/s^2'); xlabel('Time (s)');
    end
    sgtitle([method ' Mixed Frame Data']);
    fname = [base '_Task4_MixedFrame.pdf'];
    set(fig,'PaperPositionMode','auto');
    print(fig,fname,'-dpdf','-bestfit');
    print(fig,strrep(fname,'.pdf','.png'),'-dpng');
    fprintf('Mixed frames plot saved\n');
    close(fig);
end

function q_new = propagate_quaternion(q_old, w, dt)
    %PROPAGATE_QUATERNION Propagate quaternion using body angular rate.
    %   Q_NEW = PROPAGATE_QUATERNION(Q_OLD, W, DT) integrates the angular rate
    %   vector W over the timestep DT and multiplies it with the previous
    %   quaternion Q_OLD.  The returned quaternion is normalised.
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
    %   Q_OUT = QUAT_MULTIPLY(Q1, Q2) returns the quaternion product using the
    %   convention [w x y z].
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
    q_out = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
             w1*x2 + x1*w2 + y1*z2 - z1*y2;
             w1*y2 - x1*z2 + y1*w2 + z1*x2;
             w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function R = quat_to_rot(q)
    %QUAT_TO_ROT Convert quaternion to rotation matrix.
    %   R = QUAT_TO_ROT(Q) returns the 3×3 rotation matrix corresponding to the
    %   quaternion Q = [w x y z].
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R=[1-2*(qy^2+qz^2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
       2*(qx*qy+qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qw*qx);
       2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx^2+qy^2)];
end

function q = rot_to_quaternion(R)
    %ROT_TO_QUATERNION Convert rotation matrix to quaternion.
    %   Q = ROT_TO_QUATERNION(R) converts the 3×3 rotation matrix R into a
    %   quaternion Q = [w x y z].  The output is normalised and the scalar part
    %   is kept positive.
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

function g = normal_gravity_wgs84(phi, h)
%NORMAL_GRAVITY_WGS84 Normal gravity magnitude at latitude phi (rad) and height h (m)
% Somigliana + free-air correction (good enough for our use here).
if nargin < 2, h = 0; end
a = 6378137.0;               % WGS-84 semi-major axis [m]
f = 1/298.257223563;         % flattening
e2 = f*(2-f);

gamma_e = 9.7803253359;      % equatorial gravity [m/s^2]
k = 0.00193185265241;        % Somigliana constant

s = sin(phi);
s2 = s.*s;

gamma = gamma_e * (1 + k*s2) ./ sqrt(1 - e2*s2);
gamma = gamma - 3.086e-6 * h;   % free-air correction (~0.3086 mGal/m)
g = gamma;
end
