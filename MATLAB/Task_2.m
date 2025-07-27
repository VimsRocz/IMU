
function result = Task_2(imu_path, gnss_path, method)
% =========================================================================
% TASK 2: Measure the Vectors in the Body Frame
%
% This function translates Task 2 from the Python file GNSS_IMU_Fusion.py
% into MATLAB.
%
% Usage:
%   Task_2(imu_path, gnss_path, method)
% =========================================================================

if nargin < 3 || isempty(method)
    method = '';
end

%% =======================================================================
%  LOCAL FUNCTION
% =======================================================================
function M = triad_basis(v1, v2)
    t1 = v1 / norm(v1);
    t2_temp = cross(t1, v2);
    if norm(t2_temp) < 1e-10
        if abs(t1(1)) < abs(t1(2)), tmp = [1;0;0]; else, tmp = [0;1;0]; end
        t2_temp = cross(t1, tmp);
    end
    t2 = t2_temp / norm(t2_temp);
    t3 = cross(t1, t2);
    M = [t1, t2, t3];
end

if ~isfile(gnss_path)
    error('Task_2:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_2:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end


if isempty(method)
    log_tag = '';
else
    log_tag = [' (' method ')'];
end
fprintf('TASK 2%s: Measure the vectors in the body frame\n', log_tag);

% --- Configuration ---
results_dir = get_results_dir();
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
pair_tag = [imu_name '_' gnss_name];
if isempty(method)
    tag = pair_tag;
else
    tag = [pair_tag '_' method];
end

% ------------------------------------------------------------------
% Attempt to load gravity vector produced by Task 1 for this IMU/GNSS pair
% ------------------------------------------------------------------
task1_file = fullfile(results_dir, ['Task1_init_' tag '.mat']);
if ~isfile(task1_file)
    % Fallback to method-agnostic filename for backwards compatibility
    alt_file = fullfile(results_dir, ['Task1_init_' pair_tag '.mat']);
    if isfile(alt_file)
        task1_file = alt_file;
    end
end

if isfile(task1_file)
    task1_data = load(task1_file);
    if isfield(task1_data, 'g_NED')
        g_NED = task1_data.g_NED;
    else
        warning('Task_2:MissingField', ...
            'File %s does not contain g_NED. Using default gravity.', task1_file);
        g_NED = [0; 0; constants.GRAVITY];
    end
else
    warning('Task_2:MissingTask1', ...
        'Task 1 file not found: %s. Using default gravity.', task1_file);
    g_NED = [0; 0; constants.GRAVITY];
end

imu_file = imu_path;

%% ================================
% Subtask 2.1: Load and Parse IMU Data
% =================================
fprintf('\nSubtask 2.1: Loading and parsing IMU data.\n');

% Check if the IMU file exists
if ~isfile(imu_file)
    error('IMU file not found: %s', imu_file);
end

% Load IMU data. Assumes space-separated values with no header.
data = readmatrix(imu_file);

if size(data, 2) >= 8
    % Python used columns 2:5 for gyro, 5:8 for acc (0-indexed)
    % MATLAB equivalent (1-indexed): 3:5 for gyro, 6:8 for acc
    gyro_increments = data(:, 3:5); % Angular increments (rad)
    acc_increments = data(:, 6:8);  % Velocity increments (m/s)
else
    error('Unexpected data format in %s. Expected at least 8 columns.', imu_file);
end

fprintf('IMU data loaded: %d samples\n', size(data, 1));

% Estimate IMU sampling period from time column (column 2)
if size(data, 1) > 100
    dt_imu = mean(diff(data(1:100, 2)));
else
    dt_imu = 1.0 / 400.0; % Default sampling period
end

if dt_imu <= 0 || isnan(dt_imu)
    fprintf('Warning: Could not determine valid dt from data, using default 400 Hz.\n');
    dt_imu = 1.0 / 400.0;
end
fprintf('Estimated IMU sampling period: %.6f s (%.1f Hz)\n', dt_imu, 1/dt_imu);
fprintf('Estimated IMU dt: %.6f s\n', dt_imu);


%% ================================
% Subtask 2.2: Estimate Static Body-Frame Vectors
% =================================
fprintf('\nSubtask 2.2: Estimating static body-frame vectors using a low-motion interval.\n');

% --- Convert increments to rates ---
acc = acc_increments / dt_imu;  % m/s^2
gyro = gyro_increments / dt_imu; % rad/s

% --- Low-pass filter IMU data to match Python preprocessing ---
fprintf('Applying low-pass filter to accelerometer and gyroscope data.\n');
fs = 1/dt_imu;
acc_filt = low_pass_filter(acc, 10, fs);   % 10 Hz cut-off by default
gyro_filt = low_pass_filter(gyro, 10, fs);

% --- Detect a static interval automatically ---
fprintf('Detecting static interval using variance thresholds...\n');
[start_idx, end_idx] = detect_static_interval(acc_filt, gyro_filt);

% Use the automatically detected static interval
% The Python reference implementation selects a short early segment
% where the device is stationary.  Re-using that here avoids
% overestimating biases when the whole dataset is treated as static.

% --- Calculate static vectors from the interval ---
N_static = end_idx - start_idx + 1;
static_acc_row = mean(acc_filt(start_idx:end_idx, :), 1);
static_gyro_row = mean(gyro_filt(start_idx:end_idx, :), 1);
% Use helper to compute raw mean biases (unfiltered) for later use
[acc_meas, gyro_meas] = compute_biases(acc, gyro, start_idx, end_idx);
% Use population variance (w=1) for final statistics
acc_var = var(acc_filt(start_idx:end_idx, :), 1, 1);
gyro_var = var(gyro_filt(start_idx:end_idx, :), 1, 1);

fprintf('Static interval found: samples %d to %d (length %d samples)\n', start_idx, end_idx, N_static);
fprintf('  Accel variance: [%.4g %.4g %.4g]\n', acc_var);
fprintf('  Gyro  variance: [%.4g %.4g %.4g]\n', gyro_var);

% Append static interval information to triad_init_log.txt (mirrors Python)
log_file = fullfile(results_dir, 'triad_init_log.txt');
log_fid = fopen(log_file, 'a');
if log_fid ~= -1
    fprintf(log_fid, '%s: static_idx=%d-%d acc_var=[%.4g %.4g %.4g] gyro_var=[%.4g %.4g %.4g]\n', ...
            imu_file, start_idx, end_idx, acc_var, gyro_var);
    fclose(log_fid);
else
    warning('Task_2:LogFile', 'Could not open %s for writing.', log_file);
end

% Compute duration of the static portion and compare with total dataset length
static_duration = N_static * dt_imu;
total_duration  = size(acc_filt, 1) * dt_imu;
ratio_static = static_duration / total_duration;
fprintf('Static interval duration: %.2f s of %.2f s total (%.1f%%)\n', ...
        static_duration, total_duration, ratio_static*100);
if ratio_static > 0.90
    warning(['Static interval covers %.1f%% of the dataset. Verify motion data ' ...
            'or adjust detection thresholds.'], ratio_static*100);
end

g_norm = norm(static_acc_row);
fprintf('Estimated gravity magnitude from IMU: %.4f m/s^2 (expected ~%.2f)\n', ...
        g_norm, norm(g_NED));
is_grav_ok = validate_gravity_vector(acc_filt, start_idx, end_idx);
if ~is_grav_ok
    warning('Measured gravity magnitude deviates from expected value.');
end

% --- Simple accelerometer scale calibration ---
scale_factor = 1.0;
if g_norm > 1.0
    scale_factor = norm(g_NED) / g_norm;
end

if abs(scale_factor - 1.0) > 0.05
    fprintf('Applying accelerometer scale factor: %.4f\n', scale_factor);
    static_acc_row = static_acc_row * scale_factor;
end

%% ================================
% Subtask 2.3: Define Gravity and Earth Rate in Body Frame
% (Gravity vector scaled to match g_NED magnitude)
% ================================
fprintf('\nSubtask 2.3: Defining gravity and Earth rotation rate in the body frame.\n');

% By convention, vectors are column vectors. mean() returns a row, so we transpose it.
g_body_raw = -static_acc_row';
g_mag = norm(g_body_raw);
g_body = (g_body_raw / g_mag) * norm(g_NED);   % Normalize then scale to measured gravity
g_body_scaled = g_body;                  % explicitly store scaled gravity
%% Compute Earth rotation in body frame using initial latitude
% Load a GNSS sample to estimate the latitude so the expected
% Earth rotation vector can be removed from the gyroscope bias.
try
    gnss_tbl = readtable(gnss_path);
    valid_idx = find((gnss_tbl.X_ECEF_m ~= 0) | (gnss_tbl.Y_ECEF_m ~= 0) | ...
                     (gnss_tbl.Z_ECEF_m ~= 0), 1, 'first');
    if isempty(valid_idx)
        error('No valid ECEF coordinates found in GNSS file.');
    end
    [lat_deg, lon_deg, ~] = ecef2geodetic(gnss_tbl.X_ECEF_m(valid_idx), ...
                                         gnss_tbl.Y_ECEF_m(valid_idx), ...
                                         gnss_tbl.Z_ECEF_m(valid_idx));
catch ME
    warning('Failed to read latitude from GNSS: %s', ME.message);
    lat_deg = 0; lon_deg = 0;
end
lat_rad = deg2rad(lat_deg);
omega_E = constants.EARTH_RATE;
omega_ie_NED = omega_E * [cos(lat_rad); 0; -sin(lat_rad)];

% Use TRIAD with the measured vectors to obtain a rough attitude so that
% the expected Earth rotation in the body frame can be determined.
v1_B = -static_acc_row'/norm(static_acc_row);   % accelerometer measures -g
v2_B = static_gyro_row'/norm(static_gyro_row);
v1_N = g_NED/norm(g_NED);
v2_N = omega_ie_NED/norm(omega_ie_NED);
M_body = triad_basis(v1_B, v2_B);
M_ned  = triad_basis(v1_N, v2_N);
C_B_N = M_ned * M_body';       % body->NED rotation
omega_ie_body = C_B_N' * omega_ie_NED; % expected earth rotation in body frame

% Biases computed over the automatically detected static interval
% Use the same samples identified earlier for gravity estimation
static_idx = start_idx:end_idx;
static_acc  = acc_meas';
static_gyro = gyro_meas';

% Bias definitions consistent with Python implementation
accel_bias = static_acc - (-g_body);  % accel measurement minus expected
gyro_bias  = static_gyro - omega_ie_body;

fprintf('Estimated accel_bias = [% .6f % .6f % .6f]\n', accel_bias);
fprintf('Estimated gyro_bias  = [% .6e % .6e % .6e]\n', gyro_bias);
fprintf('Gravity vector (body): [% .8e % .8e % .8e]\n', g_body);
fprintf('Earth rotation (body): [% .8e % .8e % .8e]\n', omega_ie_body);

fprintf('Gravity vector in body frame (g_body):           [%.4f; %.4f; %.4f] m/s^2\n', g_body);
fprintf('Earth rotation rate in body frame (omega_ie_body): [%.6e; %.6e; %.6e] rad/s\n', omega_ie_body);
fprintf('Estimated accelerometer bias: [%.4f %.4f %.4f] m/s^2\n', accel_bias);
fprintf('Estimated gyroscope bias:     [%.6e %.6e %.6e] rad/s\n', gyro_bias);

%% ================================
% Subtask 2.4: Validate and Print Body-Frame Vectors
% =================================
fprintf('\nSubtask 2.4: Validating measured vectors in the body frame.\n');
expected_omega_mag = constants.EARTH_RATE; % rad/s

assert(isequal(size(g_body), [3, 1]), 'g_body must be a 3x1 column vector.');
assert(isequal(size(omega_ie_body), [3, 1]), 'omega_ie_body must be a 3x1 column vector.');

if norm(g_body) < 0.1 * norm(g_NED)
    warning('Gravity magnitude is very low; check accelerometer or static assumption.');
end
if norm(omega_ie_body) < 0.5 * expected_omega_mag
    warning('Earth rotation rate is low; check gyroscope or static assumption.');
end

fprintf('Magnitude of g_body:         %.6f m/s^2 (expected ~%.2f m/s^2)\n', ...
        norm(g_body), norm(g_NED));
fprintf('Magnitude of omega_ie_body:  %.6e rad/s (expected ~%.2e rad/s)\n', norm(omega_ie_body), constants.EARTH_RATE);

fprintf('\n==== Measured Vectors in the Body Frame ====\n');
fprintf('Measured gravity vector (g_body):        [%.4f, %.4f, %.4f]'' m/s^2\n', g_body);
fprintf('Measured Earth rotation (omega_ie_body): [%.4e, %.4e, %.4e]'' rad/s\n', omega_ie_body);
fprintf('\nNote: These are physical vectors expressed in the body frame (sensor axes).\n');
fprintf('From accelerometer (assuming static IMU): a_measured = -g_body \n');
fprintf('From gyroscope (assuming static IMU):     w_measured = omega_ie_body \n');

% Save results for later tasks
save(fullfile(results_dir, ['Task2_body_' tag '.mat']), 'dt_imu', 'g_body', 'g_body_scaled', 'omega_ie_body', 'accel_bias', 'gyro_bias');
fprintf('Body-frame vectors and biases saved to %s\n', fullfile(results_dir, ['Task2_body_' tag '.mat']));

% Return results and store in base workspace
result = struct('dt_imu', dt_imu, 'g_body', g_body, 'g_body_scaled', g_body_scaled, ...
                'omega_ie_body', omega_ie_body, 'accel_bias', accel_bias, ...
                'gyro_bias', gyro_bias);
assignin('base', 'task2_results', result);

end
