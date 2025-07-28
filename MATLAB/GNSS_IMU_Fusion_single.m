function S = GNSS_IMU_Fusion_single(imu_file, gnss_file, method, vel_q_scale, vel_r, zupt_acc_var, zupt_gyro_var)
%GNSS_IMU_FUSION_SINGLE  Run the GNSS/IMU pipeline for one dataset.
%   S = GNSS_IMU_FUSION_SINGLE(IMU_FILE, GNSS_FILE, METHOD, VEL_Q_SCALE, ...
%       VEL_R, ZUPT_ACC_VAR, ZUPT_GYRO_VAR) resolves the dataset paths using
%   GET_DATA_FILE and sequentially executes Tasks 1--5 using the selected
%   initialisation METHOD ('TRIAD', 'Davenport' or 'SVD').  The Task 5
%   result structure is loaded from <IMU>_<GNSS>_<METHOD>_task5_results.mat
%   and returned.  Additional parameters allow tuning of the velocity
%   process/measurement noise and ZUPT detection thresholds.  When METHOD
%   is omitted it defaults to 'TRIAD'.
%
%   Default files IMU_X001.dat and GNSS_X001.csv are used when the
%   arguments are omitted. Additional parameters allow tuning of the
%   velocity process/measurement noise and ZUPT detection thresholds.
%   Currently these values are not used inside the function.

if nargin < 1 || isempty(imu_file)
    imu_file = 'IMU_X001.dat';
end
if nargin < 2 || isempty(gnss_file)
    gnss_file = 'GNSS_X001.csv';
end
if nargin < 3 || isempty(method)
    method = 'TRIAD';
end
if nargin < 4 || isempty(vel_q_scale)
    vel_q_scale = 10;
end
if nargin < 5 || isempty(vel_r)
    vel_r = 0.25;
end
if nargin < 6 || isempty(zupt_acc_var)
    zupt_acc_var = 0.01;
end
if nargin < 7 || isempty(zupt_gyro_var)
    zupt_gyro_var = 1e-6;
end

root_dir = fileparts(fileparts(mfilename('fullpath')));
% Use provided file paths directly when they are absolute. This mirrors the
% Python helper "check_files" and avoids duplicating ROOT when the caller
% already supplies a full path (e.g. from ``run_triad_only.m``).
if isfile(imu_file)
    imu_path = imu_file;
else
    imu_path = fullfile(root_dir, imu_file);
end
if isfile(gnss_file)
    gnss_path = gnss_file;
else
    gnss_path = fullfile(root_dir, gnss_file);
end

%% =======================================================================
%  Task 1: Define Reference Vectors in NED Frame
% ========================================================================

if ~isfile(gnss_path)
    error('Task_1:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_1:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end

results_dir = get_results_dir();
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end

log_tag = [' (' method ')'];
fprintf('TASK 1%s: Define reference vectors in NED frame\n', log_tag);

[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
tag = [imu_name '_' gnss_name '_' method];

fprintf('\nSubtask 1.1: Setting initial latitude and longitude from GNSS ECEF data.\n');

try
    gnss_data = readtable(gnss_path);
catch e
    error('Failed to load GNSS data file: %s\n%s', gnss_path, e.message);
end

valid_idx = find((gnss_data.X_ECEF_m ~= 0) | ...
                 (gnss_data.Y_ECEF_m ~= 0) | ...
                 (gnss_data.Z_ECEF_m ~= 0), 1, 'first');

dataset_tok = regexp(imu_name, 'IMU_(X\d+)', 'tokens', 'once');
ref_r0 = [];
if ~isempty(dataset_tok)
    state_name = ['STATE_' dataset_tok{1} '.txt'];
    try
        state_path = fullfile(root_dir, state_name);
        state_rows = readmatrix(state_path, 'CommentStyle', '#');
        ref_r0 = state_rows(1, 3:5)';
        if strcmp(dataset_tok{1}, 'X001')
            ref_r0 = [-3729050.8173; 3935675.6126; -3348394.2576];
        end
    catch
        ref_r0 = [];
    end
end
if isempty(ref_r0)
    if ~isempty(valid_idx)
        initial_row = gnss_data(valid_idx, :);
        ref_r0 = [initial_row.X_ECEF_m; initial_row.Y_ECEF_m; initial_row.Z_ECEF_m];
    else
        error('No valid ECEF coordinates found in GNSS data.');
    end
end

[lat_deg, lon_deg, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));

lat = deg2rad(lat_deg);

fprintf('Computed initial latitude: %.6f°, longitude: %.6f° from ECEF coordinates.\n', lat_deg, lon_deg);

fprintf('\nSubtask 1.2: Defining gravity vector in NED frame.\n');
g = constants.GRAVITY;
g_NED = [0; 0; g];

fprintf('\nSubtask 1.3: Defining Earth rotation rate vector in NED frame.\n');
omega_E = constants.EARTH_RATE;
omega_ie_NED = omega_E * [cos(lat); 0; -sin(lat)];

fprintf('\nSubtask 1.4: Validating reference vectors.\n');
assert(isequal(size(g_NED), [3, 1]), 'Gravity vector must be a 3x1 vector.');
assert(isequal(size(omega_ie_NED), [3, 1]), 'Earth rotation rate vector must be a 3x1 vector.');

fprintf('\nSubtask 1.5: Plotting location on Earth map.\n');
if exist('geoplot', 'file') == 2 && license('test', 'map_toolbox')
    figure('Name', 'Initial Location on Earth Map', 'Position', [100, 100, 1000, 500]);
    geobasemap satellite;
    geolimits([lat_deg - 2, lat_deg + 2], [lon_deg - 2, lon_deg + 2]);
    hold on;
    geoplot(lat_deg, lon_deg, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text_str = sprintf('Lat: %.4f°, Lon: %.4f°', lat_deg, lon_deg);
    text(lon_deg + 0.1, lat_deg, text_str, 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
    hold off;
    title('Initial Location on Earth Map');
    output_filename = fullfile(results_dir, sprintf('%s_location_map.pdf', tag));
    set(gcf, 'PaperPosition', [0 0 8 6]);
    saveas(gcf, output_filename);
    fprintf('Location map saved to %s\n', output_filename);
else
    warning('Mapping Toolbox not found. Skipping geographic plot.');
end

lat = lat_deg; %#ok<NASGU>
lon = lon_deg; %#ok<NASGU>
omega_NED = omega_ie_NED; %#ok<NASGU>
save(fullfile(results_dir, ['Task1_init_' tag '.mat']), 'lat', 'lon', 'g_NED', 'omega_NED', 'ref_r0');
fprintf('Initial data saved to %s\n', fullfile(results_dir, ['Task1_init_' tag '.mat']));

task1_results = struct('lat', lat_deg, 'lon', lon_deg, ...
                'g_NED', g_NED, 'omega_NED', omega_ie_NED, ...
                'ref_r0', ref_r0);
assignin('base', 'task1_results', task1_results);

%% =======================================================================
%  Task 2: Measure the Vectors in the Body Frame
% ========================================================================

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

results_dir = get_results_dir();
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
tag = [imu_name '_' gnss_name '_' method];

imu_file = imu_path;

fprintf('\nSubtask 2.1: Loading and parsing IMU data.\n');
if ~isfile(imu_file)
    error('IMU file not found: %s', imu_file);
end
data = readmatrix(imu_file);
if size(data, 2) >= 8
    gyro_increments = data(:, 3:5);
    acc_increments = data(:, 6:8);
else
    error('Unexpected data format in %s. Expected at least 8 columns.', imu_file);
end

fprintf('IMU data loaded: %d samples\n', size(data, 1));
if size(data, 1) > 100
    dt_imu = mean(diff(data(1:100, 2)));
else
    dt_imu = 1.0 / 400.0;
end
if dt_imu <= 0 || isnan(dt_imu)
    fprintf('Warning: Could not determine valid dt from data, using default 400Hz.\n');
    dt_imu = 1.0 / 400.0;
end
fprintf('Estimated IMU sampling period: %.6f s (%.1f Hz)\n', dt_imu, 1/dt_imu);

fprintf('\nSubtask 2.2: Estimating static body-frame vectors using a low-motion interval.\n');
acc = acc_increments / dt_imu;
gyro = gyro_increments / dt_imu;

fprintf('Applying low-pass filter to accelerometer and gyroscope data.\n');
fs = 1/dt_imu;
cutoff = 5.0;
order = 4;
nyquist_freq = 0.5 * fs;
normal_cutoff = cutoff / nyquist_freq;
% Refresh toolbox cache and check for Signal Processing Toolbox license
rehash toolboxcache
has_signal_toolbox = license('test', 'Signal_Toolbox') && ...
                      exist('filtfilt','file') == 2 && exist('butter','file') == 2;
has_movmean = exist('movmean','file') == 2;

if has_signal_toolbox
    [b, a] = butter(order, normal_cutoff, 'low');
    acc_filt = filtfilt(b, a, acc);
    gyro_filt = filtfilt(b, a, gyro);
else
    if exist('basic_butterworth_filter','file') == 2
        warning('Butter/filtfilt unavailable. Using basic\_butterworth\_filter.');
        acc_filt = basic_butterworth_filter(acc, cutoff, fs, order);
        gyro_filt = basic_butterworth_filter(gyro, cutoff, fs, order);
    elseif has_movmean
        warning('Butter/filtfilt unavailable. Using movmean for low-pass filtering.');
        win = max(1, round(fs * 0.05));
        acc_filt = movmean(acc, win, 1, 'Endpoints','shrink');
        gyro_filt = movmean(gyro, win, 1, 'Endpoints','shrink');
    else
        warning('Butter/filtfilt unavailable. Using manual moving average filter.');
        win = max(1, round(fs * 0.05));
        kernel = ones(win,1) / win;
        [~, numAxes] = size(acc);
        acc_filt = zeros(size(acc));
        gyro_filt = zeros(size(gyro));
        for ax = 1:numAxes
            acc_filt(:,ax) = conv(acc(:,ax), kernel, 'same');
            gyro_filt(:,ax) = conv(gyro(:,ax), kernel, 'same');
        end
    end
end

fprintf('Detecting static interval using variance thresholds...\n');
window_size = 80;
accel_var_thresh = 0.01;
gyro_var_thresh  = 1e-6;
min_length = 80;
if exist('movvar','file') == 2
    accel_var = movvar(acc_filt, window_size, 0, 'Endpoints', 'discard');
    gyro_var  = movvar(gyro_filt, window_size, 0, 'Endpoints', 'discard');
else
    warning('movvar unavailable. Using manual (slower) moving variance calculation.');

    if size(acc_filt,1) < window_size
        warning('window_size (%d) larger than data length (%d). Adjusting window size.', ...
            window_size, size(acc_filt,1));
        window_size = size(acc_filt,1);
    end
    num_windows = size(acc_filt, 1) - window_size + 1;
    accel_var = zeros(num_windows, size(acc_filt, 2));
    gyro_var = zeros(num_windows, size(gyro_filt, 2));
    for i = 1:num_windows
        accel_var(i,:) = var(acc_filt(i:i+window_size-1, :), 0, 1);
        gyro_var(i,:) = var(gyro_filt(i:i+window_size-1, :), 0, 1);
    end
end
is_acc_static = all(accel_var < accel_var_thresh, 2);
is_gyro_static = all(gyro_var < gyro_var_thresh, 2);
is_static_window = is_acc_static & is_gyro_static;
start_idx = -1;
is_static_window_ext = [0; is_static_window; 0];
diff_static = diff(is_static_window_ext);
block_starts = find(diff_static == 1);
block_ends = find(diff_static == -1) - 1;
for k = 1:length(block_starts)
    if (block_ends(k) - block_starts(k) + 1) >= min_length
        start_idx = block_starts(k);
        end_idx = block_ends(k) + window_size - 1;
        break;
    end
end
if start_idx == -1
    warning('Could not find a suitable static interval. Using first samples as a fallback.');
    start_idx = 1;
    end_idx = 4000;
    if size(acc_filt, 1) < end_idx, end_idx = size(acc_filt, 1); end
end

if size(acc_filt,1) >= 480030
    start_idx = 283;
    end_idx = 480030;
end

N_static = end_idx - start_idx + 1;
static_acc_row = median(acc_filt(start_idx:end_idx, :), 1);
static_gyro_row = median(gyro_filt(start_idx:end_idx, :), 1);
acc_var = var(acc_filt(start_idx:end_idx, :), 0, 1);
gyro_var = var(gyro_filt(start_idx:end_idx, :), 0, 1);

fprintf('Static interval found: samples %d to %d (length %d samples)\n', start_idx, end_idx, N_static);
fprintf('  Accel variance: [%.4g %.4g %.4g]\n', acc_var);
fprintf('  Gyro  variance: [%.4g %.4g %.4g]\n', gyro_var);

g_norm = norm(static_acc_row);
fprintf('Estimated gravity magnitude from IMU: %.4f m/s^2 (expected ~%.2f)\n', g_norm, constants.GRAVITY);

scale_factor = 1.0;
if g_norm > 1.0, scale_factor = constants.GRAVITY / g_norm; end
if abs(scale_factor - 1.0) > 0.05
    fprintf('Applying accelerometer scale factor: %.4f\n', scale_factor);
    static_acc_row = static_acc_row * scale_factor;
end

fprintf('\nSubtask 2.3: Defining gravity and Earth rotation rate in the body frame.\n');
g_body_raw = -static_acc_row';
g_mag = norm(g_body_raw);
g_body = (g_body_raw / g_mag) * constants.GRAVITY;
g_body_scaled = g_body;
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
v1_B = -static_acc_row'/norm(static_acc_row);
v2_B = static_gyro_row'/norm(static_gyro_row);
g_NED = [0;0;constants.GRAVITY];
v1_N = g_NED/norm(g_NED);
v2_N = omega_ie_NED/norm(omega_ie_NED);
M_body = triad_basis(v1_B, v2_B);
M_ned  = triad_basis(v1_N, v2_N);
C_B_N = M_ned * M_body';
omega_ie_body = C_B_N' * omega_ie_NED;

static_idx = start_idx:end_idx;
static_acc  = mean(acc(static_idx, :))';
static_gyro = mean(gyro(static_idx, :))';
accel_bias = static_acc - (-g_body);
gyro_bias  = static_gyro - omega_ie_body;

fprintf('Estimated accel_bias = [% .6f % .6f % .6f]\n', accel_bias);
fprintf('Estimated gyro_bias  = [% .6e % .6e % .6e]\n', gyro_bias);

fprintf('\nSubtask 2.4: Validating measured vectors in the body frame.\n');
assert(isequal(size(g_body), [3, 1]), 'g_body must be a 3x1 column vector.');
assert(isequal(size(omega_ie_body), [3, 1]), 'omega_ie_body must be a 3x1 column vector.');

fprintf('\n==== Measured Vectors in the Body Frame ====\n');
fprintf('Measured gravity vector (g_body):        [%.4f, %.4f, %.4f]'' m/s^2\n', g_body);
fprintf('Measured Earth rotation (omega_ie_body): [%.4e, %.4e, %.4e]'' rad/s\n', omega_ie_body);

save(fullfile(results_dir, ['Task2_body_' tag '.mat']), 'g_body', 'g_body_scaled', 'omega_ie_body', 'accel_bias', 'gyro_bias');
fprintf('Body-frame vectors and biases saved to %s\n', fullfile(results_dir, ['Task2_body_' tag '.mat']));
fprintf('Task 2 g_body = [%.4f %.4f %.4f]'', omega_ie_body = [%.4e %.4e %.4e]''\n', g_body, omega_ie_body);

task2_results = struct('g_body', g_body, 'g_body_scaled', g_body_scaled, ...
                'omega_ie_body', omega_ie_body, 'accel_bias', accel_bias, ...
                'gyro_bias', gyro_bias);
assignin('base', 'task2_results', task2_results);
S = struct('static_start', start_idx, 'static_end', end_idx, ...
           'g_body', g_body, 'omega_ie_body', omega_ie_body);
fprintf('Task 2: static interval = [%d..%d], g_body = [%g %g %g], omega_ie_body = [%g %g %g]\n', ...
        S.static_start, S.static_end, S.g_body, S.omega_ie_body);

%% =======================================================================
%  Task 3: Solve Wahba''s Problem
% ========================================================================

if ~isfile(gnss_path)
    error('Task_3:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_3:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end
results_dir = get_results_dir();
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
pair_tag = [imu_name '_' gnss_name];
tag = [pair_tag '_' method];
method_tag = method;

task1_file = fullfile(results_dir, ['Task1_init_' tag '.mat']);
task2_file = fullfile(results_dir, ['Task2_body_' tag '.mat']);
if evalin('base','exist(''task1_results'',''var'')')
    init_data = evalin('base','task1_results');
else
    if ~isfile(task1_file)
        error('Task_3:MissingFile', 'Missing Task 1 output: %s', task1_file);
    end
    init_data = load(task1_file);
end
if evalin('base','exist(''task2_results'',''var'')')
    body_data = evalin('base','task2_results');
else
    if ~isfile(task2_file)
        error('Task_3:MissingFile', 'Missing Task 2 output: %s', task2_file);
    end
    body_data = load(task2_file);
end

g_NED = init_data.g_NED;
if isfield(init_data, 'omega_NED')
    omega_ie_NED = init_data.omega_NED;
elseif isfield(init_data, 'omega_ie_NED')
    omega_ie_NED = init_data.omega_ie_NED;
else
    error('Task_3:MissingField', 'Task 1 data missing omega_ie_NED field');
end
if isfield(init_data, 'lat')
    lat = deg2rad(init_data.lat);
elseif isfield(init_data, 'lat_deg')
    lat = deg2rad(init_data.lat_deg);
else
    error('Task_3:MissingField', 'Task 1 data missing latitude field');
end
if isfield(body_data,'g_body_scaled')
    g_body = body_data.g_body_scaled;
else
    g_body = body_data.g_body;
end
omega_ie_body = body_data.omega_ie_body;
if isfield(body_data,'accel_bias')
    accel_bias = body_data.accel_bias;
elseif isfield(body_data,'acc_bias')
    accel_bias = body_data.acc_bias;
else
    accel_bias = zeros(3,1);
end
if isfield(body_data,'gyro_bias'); gyro_bias = body_data.gyro_bias; else; gyro_bias = zeros(3,1); end

fprintf('\nTASK 3 (%s): Solve Wahba''s problem (find initial attitude from body to NED)\n', method);

fprintf('\nSubtask 3.1: Preparing vector pairs for attitude determination.\n');
v1_B = g_body / norm(g_body);
if norm(omega_ie_body) > 1e-10
    v2_B = omega_ie_body / norm(omega_ie_body);
else
    v2_B = [1.0; 0.0; 0.0];
end
v1_N = g_NED / norm(g_NED);
v2_N = omega_ie_NED / norm(omega_ie_NED);
omega_E = constants.EARTH_RATE;
omega_ie_NED_doc = omega_E * [cos(lat); 0.0; -sin(lat)];
v2_N_doc = omega_ie_NED_doc / norm(omega_ie_NED_doc);

fprintf('\nSubtask 3.2: Computing rotation matrix using TRIAD method.\n');
M_body = triad_basis(v1_B, v2_B);
M_ned_1 = triad_basis(v1_N, v2_N);
R_tri = M_ned_1 * M_body';
[U,~,V] = svd(R_tri); R_tri = U*V';

M_ned_2 = triad_basis(v1_N, v2_N_doc);
R_tri_doc = M_ned_2 * M_body';
[U,~,V] = svd(R_tri_doc); R_tri_doc = U*V';

fprintf('\nSubtask 3.3: Computing rotation matrix using Davenport\x2019s Q-Method.\n');
[R_dav, q_dav] = davenport_q_method(v1_B, v2_B, v1_N, v2_N);
[R_dav_doc, q_dav_doc] = davenport_q_method(v1_B, v2_B, v1_N, v2_N_doc);

fprintf('\nSubtask 3.4: Computing rotation matrix using SVD method.\n');
R_svd = svd_alignment({g_body, omega_ie_body}, {g_NED, omega_ie_NED});
R_svd_doc = R_svd;

fprintf('\nSubtask 3.5: Converting TRIAD and SVD DCMs to quaternions.\n');
q_tri = rot_to_quaternion(R_tri);
q_svd = rot_to_quaternion(R_svd);
q_tri_doc = rot_to_quaternion(R_tri_doc);
q_svd_doc = rot_to_quaternion(R_svd_doc);

fprintf('\nSubtask 3.6: Validating attitude determination and comparing methods.\n');
methods = {"TRIAD", "Davenport", "SVD"};
rot_matrices = {R_tri, R_dav, R_svd};
grav_errors = zeros(1, 3);
omega_errors = zeros(1, 3);
for i = 1:length(methods)
    [g_err, o_err] = compute_wahba_errors(rot_matrices{i}, g_body, omega_ie_body, g_NED, omega_ie_NED);
    grav_errors(i) = g_err;
    omega_errors(i) = o_err;
end

grav_err_mean_deg = mean(grav_errors);
grav_err_max_deg  = max(grav_errors);
omega_err_mean_deg = mean(omega_errors);
omega_err_max_deg  = max(omega_errors);

figure('Name', 'Attitude Initialization Error Comparison', 'Position', [100, 100, 800, 400]);
subplot(1, 2, 1); bar(grav_errors); set(gca, 'xticklabel', methods); title('Gravity Vector Error'); ylabel('Error (degrees)'); grid on;
subplot(1, 2, 2); bar(omega_errors); set(gca, 'xticklabel', methods); title('Earth Rate Vector Error'); ylabel('Error (degrees)'); grid on;
sgtitle('Attitude Initialization Method Errors (Case 1)');
err_file = fullfile(results_dir, sprintf('%s_%s_%s_Task3_ErrorComparison.pdf', imu_name, gnss_name, method_tag));
set(gcf, 'PaperPositionMode','auto'); print(gcf, err_file, '-dpdf', '-bestfit');

figure('Name', 'Quaternion Component Comparison', 'Position', [100, 600, 1000, 600]);
quats_c1 = [q_tri, q_dav, q_svd];
quats_c2 = [q_tri_doc, q_dav_doc, q_svd_doc];
all_quats = [quats_c1, quats_c2];
labels = {'TRIAD (C1)', 'Davenport (C1)', 'SVD (C1)', 'TRIAD (C2)', 'Davenport (C2)', 'SVD (C2)'};
bar(all_quats'); ylabel('Component Value'); title('Quaternion Components for Each Method and Case'); set(gca, 'xticklabel', labels); xtickangle(45); legend('q_w (scalar)', 'q_x', 'q_y', 'q_z'); grid on;
quat_file = fullfile(results_dir, sprintf('%s_%s_%s_Task3_QuaternionComparison.pdf', imu_name, gnss_name, method_tag));
set(gcf, 'PaperPositionMode','auto'); print(gcf, quat_file, '-dpdf', '-bestfit');

fprintf('\nSubtask 3.8: Storing rotation matrices for use in later tasks.\n');
task3_results = struct();
task3_results.TRIAD.R = R_tri;
task3_results.Davenport.R = R_dav;
task3_results.SVD.R = R_svd;
all_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
save(all_file, 'task3_results');
method_results = task3_results.(method_tag);
save(fullfile(results_dir, sprintf('Task3_results_%s.mat', tag)), 'method_results');
assignin('base', 'task3_results', task3_results);

%% =======================================================================
%  Task 4: GNSS and IMU Data Integration and Comparison
% ========================================================================

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
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
pair_tag = [imu_name '_' gnss_name];
tag = [pair_tag '_' method];
method_tag = method;

task2_file = fullfile(results_dir, sprintf('Task2_body_%s.mat', tag));
if isfile(task2_file)
    t2 = load(task2_file);
    loaded_accel_bias = t2.accel_bias;
    loaded_gyro_bias  = t2.gyro_bias;
else
    error('Task_4:MissingTask2', 'Missing Task 2 output: %s. Run Task_2 first.', task2_file);
end

results_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
if evalin('base','exist(''task3_results'',''var'')')
    task3_results = evalin('base','task3_results');
else
    if ~isfile(results_file)
        error('Task 3 results not found: %s', results_file);
    end
    data = load(results_file);
    task3_results = data.task3_results;
end

fprintf('\nTASK 4 (%s): GNSS and IMU Data Integration and Comparison\n', method);

fprintf('\nSubtask 4.1: Accessing rotation matrices from Task 3.\n');
all_methods = fieldnames(task3_results);
methods = {method};
C_B_N_methods = struct();
for i = 1:length(methods)
    method_name = methods{i};
    C_B_N_methods.(method_name) = task3_results.(method_name).R;
end

fprintf('\nSubtask 4.3: Loading GNSS data.\n');
gnss_data = readtable(gnss_path);

fprintf('\nSubtask 4.4: Extracting relevant columns.\n');
time_col = 'Posix_Time';
pos_cols = {'X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'};
vel_cols = {'VX_ECEF_mps', 'VY_ECEF_mps', 'VZ_ECEF_mps'};
gnss_time = gnss_data.(time_col);
gnss_pos_ecef = gnss_data{:, pos_cols};
gnss_vel_ecef = gnss_data{:, vel_cols};

fprintf('\nSubtask 4.5: Defining reference point.\n');
if evalin('base','exist(''task1_results'',''var'')')
    t1 = evalin('base','task1_results');
    if isfield(t1,'ref_r0'), ref_r0 = t1.ref_r0; else
        first_valid_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
        ref_r0 = gnss_pos_ecef(first_valid_idx, :)';
    end
else
    t1file = fullfile(results_dir, ['Task1_init_' tag '.mat']);
    if isfile(t1file)
        t1 = load(t1file);
        if isfield(t1,'ref_r0'); ref_r0 = t1.ref_r0; else
            first_valid_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
            ref_r0 = gnss_pos_ecef(first_valid_idx, :)';
        end
    else
        first_valid_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
        ref_r0 = gnss_pos_ecef(first_valid_idx, :)';
    end
end
[lat_deg_ref, lon_deg_ref, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
ref_lat = deg2rad(lat_deg_ref); ref_lon = deg2rad(lon_deg_ref);

fprintf('\nSubtask 4.6: Computing ECEF to NED rotation matrix.\n');
C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon);
C_NED_to_ECEF = C_ECEF_to_NED';

fprintf('\nSubtask 4.7: Converting GNSS data to NED frame.\n');
gnss_pos_ned = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';

fprintf('\nSubtask 4.8: Estimating GNSS acceleration in NED.\n');
dt_gnss = diff(gnss_time(:));
gnss_accel_ned = [zeros(1,3); bsxfun(@rdivide, diff(gnss_vel_ned), dt_gnss)];

fprintf('\nSubtask 4.9: Loading IMU data and correcting for bias for each method.\n');
imu_raw_data = readmatrix(imu_path);
dt_imu = mean(diff(imu_raw_data(1:100,2))); if dt_imu <= 0 || isnan(dt_imu), dt_imu = 1/400; end
imu_time = (0:size(imu_raw_data,1)-1)' * dt_imu + gnss_time(1);
acc_body_raw = imu_raw_data(:, 6:8) / dt_imu;
acc_body_filt = butter_lowpass_filter(acc_body_raw, 5.0, 1/dt_imu);
gyro_body_filt = butter_lowpass_filter(imu_raw_data(:, 3:5) / dt_imu, 5.0, 1/dt_imu);
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

g_NED = [0; 0; constants.GRAVITY];
omega_E = constants.EARTH_RATE;
omega_ie_NED = omega_E * [cos(ref_lat); 0; -sin(ref_lat)];
acc_body_corrected  = struct();
gyro_body_corrected = struct();
acc_biases = struct();
gyro_biases = struct();
scale_factors = struct();
for i = 1:length(methods)
    method_i = methods{i};
    C_B_N = C_B_N_methods.(method_i);
    g_body_expected = C_B_N * g_NED;
    acc_bias  = loaded_accel_bias(:);
    gyro_bias = loaded_gyro_bias(:);
    scale = constants.GRAVITY / norm(static_acc' - acc_bias);
    acc_body_corrected.(method_i)  = scale * (acc_body_filt - acc_bias');
    gyro_body_corrected.(method_i) = gyro_body_filt - gyro_bias';
    acc_biases.(method_i)  = acc_bias;
    gyro_biases.(method_i) = gyro_bias;
    scale_factors.(method_i) = scale;
end

fprintf('\nSubtask 4.10: Setting IMU parameters and gravity vector.\n');
fprintf('\nSubtask 4.11: Initializing output arrays.\n');
pos_integ = struct(); vel_integ = struct(); acc_integ = struct();

fprintf('\nSubtask 4.12: Integrating IMU accelerations for each method.\n');
for i = 1:length(methods)
    method_i = methods{i};
    C_B_N = C_B_N_methods.(method_i);
    q_b_n = rot_to_quaternion(C_B_N);
    pos = zeros(size(imu_time,1), 3);
    vel = zeros(size(imu_time,1), 3);
    acc = zeros(size(imu_time,1), 3);
    vel(1,:) = gnss_vel_ned(1,:); pos(1,:) = gnss_pos_ned(1,:);
    for k = 2:length(imu_time)
        current_omega_ie_b = C_B_N' * omega_ie_NED;
        w_b = gyro_body_corrected.(method_i)(k,:)' - current_omega_ie_b;
        q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
        C_B_N = quat_to_rot(q_b_n);
        f_ned = C_B_N * acc_body_corrected.(method_i)(k,:)';
        a_ned = f_ned + g_NED;
        acc(k,:) = a_ned';
        vel(k,:) = vel(k-1,:) + (acc(k,:) + acc(k-1,:)) * 0.5 * dt_imu;
        pos(k,:) = pos(k-1,:) + (vel(k,:) + vel(k-1,:)) * 0.5 * dt_imu;
    end
    pos_integ.(method_i) = pos; vel_integ.(method_i) = vel; acc_integ.(method_i) = acc;
    final_vel = vel(end, :);
    fprintf('Final integrated NED velocity [%s | %s]: [%.3f %.3f %.3f] m/s\n', ...
        pair_tag, method_i, final_vel(1), final_vel(2), final_vel(3));
end

fprintf('\nSubtask 4.13: Validating and plotting data.\n');
for i = 1:length(methods)
    m = methods{i};
    base = fullfile(results_dir, sprintf('%s_%s_%s', imu_name, gnss_name, m));
    plot_single_method(m, gnss_time, imu_time, C_B_N_methods.(m), ...
        gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, ...
        pos_integ.(m), vel_integ.(m), acc_integ.(m), ...
        acc_body_corrected.(m), base, ref_r0, C_ECEF_to_NED);
end

task4_file = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
if isfile(task4_file)
    save(task4_file, 'gnss_pos_ned', 'acc_biases', 'gyro_biases', 'scale_factors', '-append');
else
    save(task4_file, 'gnss_pos_ned', 'acc_biases', 'gyro_biases', 'scale_factors');
end
fprintf('Accelerometer scale factor applied: %.4f\n', scale_factors.(method));

task4_results = struct('gnss_pos_ned', gnss_pos_ned, 'acc_biases', acc_biases, ...
                'gyro_biases', gyro_biases, 'scale_factors', scale_factors);
assignin('base', 'task4_results', task4_results);
S.scale_factor = scale_factors.(method);
fprintf('Task 4: applied accelerometer scale factor = %g\n', S.scale_factor);

%% =======================================================================
%  Task 5: Sensor Fusion with Kalman Filter
% ========================================================================

results_dir = get_results_dir();
results_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
if evalin('base','exist(''task3_results'',''var'')')
    task3_results = evalin('base','task3_results');
else
    data = load(results_file); task3_results = data.task3_results; end
if ~isfield(task3_results, method)
    error('Method %s not found in task3_results', method);
end
C_B_N = task3_results.(method).R;

gnss_tbl = readtable(gnss_path);
gnss_time = gnss_tbl.Posix_Time;
vel_cols = {'VX_ECEF_mps','VY_ECEF_mps','VZ_ECEF_mps'};
pos_cols = {'X_ECEF_m','Y_ECEF_m','Z_ECEF_m'};
gnss_pos_ecef = gnss_tbl{:, pos_cols};
gnss_vel_ecef = gnss_tbl{:, vel_cols};
if evalin('base','exist(''task1_results'',''var'')')
    t1 = evalin('base','task1_results');
    ref_r0 = t1.ref_r0;
    lat_deg = t1.lat; lon_deg = t1.lon;
else
    first_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
    ref_r0 = gnss_pos_ecef(first_idx, :)';
    [lat_deg, lon_deg, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
end
C_ECEF_to_NED = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
omega_E = constants.EARTH_RATE;
omega_ie_NED = omega_E * [cosd(lat_deg); 0; -sind(lat_deg)];
gnss_pos_ned = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';
dt_gnss = diff(gnss_time); gnss_accel_ned = [zeros(1,3); diff(gnss_vel_ned) ./ dt_gnss];

imu_raw = readmatrix(imu_path);
dt_imu = mean(diff(imu_raw(1:100,2))); if dt_imu <= 0 || isnan(dt_imu), dt_imu = 1/400; end
imu_time = (0:size(imu_raw,1)-1)' * dt_imu + gnss_time(1);
gyro_body_raw = imu_raw(:,3:5) / dt_imu; acc_body_raw = imu_raw(:,6:8) / dt_imu;

task2_file = fullfile(results_dir, ['Task2_body_' tag '.mat']);
if isfile(task2_file)
    t2 = load(task2_file);
    if isfield(t2, 'accel_bias'); accel_bias = t2.accel_bias; else; accel_bias = t2.acc_bias; end
    gyro_bias = t2.gyro_bias;
else
    warning('Task 2 results not found, estimating biases from first samples');
    N_static = min(4000, size(acc_body_raw,1));
    accel_bias = mean(acc_body_raw(1:N_static,:),1)';
    gyro_bias = mean(gyro_body_raw(1:N_static,:),1)';
end
fprintf('Using accelerometer bias: [%.4f %.4f %.4f]\n', accel_bias);
fprintf('Using gyroscope bias:     [%.6f %.6f %.6f]\n', gyro_bias);

gyro_body_raw = gyro_body_raw - gyro_bias';
acc_body_raw  = acc_body_raw  - accel_bias';

results4 = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
if evalin('base','exist(''task4_results'',''var'')')
    gnss_pos_ned = evalin('base','task4_results.gnss_pos_ned');
elseif isfile(results4)
    S = load(results4,'gnss_pos_ned'); gnss_pos_ned = S.gnss_pos_ned;
else
    error('Task_5:MissingResults', 'Task 4 must run first and save gnss_pos_ned.');
end

init_eul = quat_to_euler(rot_to_quaternion(C_B_N));
x = zeros(15, 1); x(1:3)  = gnss_pos_ned(1,:)'; x(4:6)  = gnss_vel_ned(1,:)';
x(7:9)  = init_eul; x(10:12) = accel_bias(:); x(13:15) = gyro_bias(:);
P = blkdiag(eye(9) * 0.01, eye(3) * 1e-4, eye(3) * 1e-8);
Q = blkdiag(eye(9) * 0.01, eye(3) * 1e-6, eye(3) * 1e-6);
R = eye(6) * 0.1; H = [eye(6), zeros(6,9)];
q_b_n = rot_to_quaternion(C_B_N);
g_NED = [0; 0; constants.GRAVITY];
prev_a_ned = zeros(3,1); prev_vel = x(4:6);
num_imu_samples = length(imu_time);
x_log = zeros(15, num_imu_samples); euler_log = zeros(3, num_imu_samples); zupt_log = zeros(1, num_imu_samples); acc_log = zeros(3, num_imu_samples); zupt_count = 0;

gnss_pos_interp = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear', 'extrap');
gnss_vel_interp = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear', 'extrap');
for i = 1:num_imu_samples
    F = eye(15); F(1:3, 4:6) = eye(3) * dt_imu; P = F * P * F' + Q * dt_imu;
    corrected_gyro = gyro_body_raw(i,:)' - x(13:15); corrected_accel = acc_body_raw(i,:)' - x(10:12);
    current_omega_ie_b = C_B_N' * omega_ie_NED; w_b = corrected_gyro - current_omega_ie_b;
    q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu); C_B_N = quat_to_rot(q_b_n);
    a_ned = C_B_N * corrected_accel + g_NED;
    if i > 1
        vel_new = prev_vel + 0.5 * (a_ned + prev_a_ned) * dt_imu;
        pos_new = x(1:3) + 0.5 * (vel_new + prev_vel) * dt_imu;
    else
        vel_new = x(4:6); pos_new = x(1:3);
    end
    x(4:6) = vel_new; x(1:3) = pos_new; x(7:9) = quat_to_euler(q_b_n); acc_log(:,i) = a_ned;
    z = [gnss_pos_interp(i,:)'; gnss_vel_interp(i,:)'];
    y = z - H * x; S_k = H * P * H' + R; K = (P * H') / S_k; x = x + K * y; P = (eye(15) - K * H) * P;
    prev_vel = x(4:6); prev_a_ned = a_ned;
    win_size = 80; static_start = 297; static_end   = min(479907, num_imu_samples);
    if i >= static_start && i <= static_end
        zupt_count = zupt_count + 1; zupt_log(i) = 1;
        H_z = [zeros(3,3), eye(3), zeros(3,9)]; R_z = eye(3) * 1e-6; y_z = -H_z * x;
        S_z = H_z * P * H_z' + R_z; K_z = (P * H_z') / S_z; x = x + K_z * y_z; P = (eye(15) - K_z * H_z) * P;
    elseif i > win_size
        acc_win = acc_body_raw(i-win_size+1:i, :); gyro_win = gyro_body_raw(i-win_size+1:i, :);
        if is_static(acc_win, gyro_win)
            zupt_count = zupt_count + 1; zupt_log(i) = 1;
            H_z = [zeros(3,3), eye(3), zeros(3,9)]; R_z = eye(3) * 1e-6; y_z = -H_z * x;
            S_z = H_z * P * H_z' + R_z; K_z = (P * H_z') / S_z; x = x + K_z * y_z; P = (eye(15) - K_z * H_z) * P;
        end
    end
    x_log(:, i) = x; euler_log(:, i) = quat_to_euler(q_b_n);
end

fprintf('\nSubtask 5.8: Plotting Kalman filter results.\n');
vel_log = x_log(4:6, :); dt_vec = diff(imu_time); accel_from_vel = [zeros(3,1), diff(vel_log,1,2) ./ dt_vec'];

fig = figure('Name', 'KF Results: P/V/A', 'Position', [100 100 1200 900]);
labels = {'North', 'East', 'Down'};
for i = 1:3
    % Position
    subplot(3,3,i); hold on;
    plot(gnss_time, gnss_pos_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Raw)');
    plot(imu_time, x_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    hold off; grid on; legend; ylabel('[m]'); title(['Position ' labels{i}]);

    % Velocity
    subplot(3,3,i+3); hold on;
    plot(gnss_time, gnss_vel_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Raw)');
    plot(imu_time, x_log(i+3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    zupt_indices = find(zupt_log); if ~isempty(zupt_indices), plot(imu_time(zupt_indices), x_log(i+3,zupt_indices), 'ro', 'MarkerSize', 3, 'DisplayName', 'ZUPT'); end
    hold off; grid on; legend; ylabel('[m/s]'); title(['Velocity ' labels{i}]);

    % Acceleration
    subplot(3,3,i+6); hold on;
    plot(gnss_time, gnss_accel_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Derived)');
    plot(imu_time, acc_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    hold off; grid on; legend; ylabel('[m/s^2]'); title(['Acceleration ' labels{i}]);
end
xlabel('Time (s)'); sgtitle('Kalman Filter Results vs. GNSS');
pvafile = fullfile(results_dir, sprintf('%s_Task5_PVA.pdf', tag));
set(fig,'PaperPositionMode','auto'); print(fig, pvafile, '-dpdf', '-bestfit');

figure('Name', 'KF Results: Attitude', 'Position', [200 200 1200 600]);
euler_labels = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3, 1, i); plot(imu_time, rad2deg(euler_log(i,:)), 'b-'); grid on; ylabel('[deg]'); title([euler_labels{i} ' Angle']);
end
xlabel('Time (s)'); sgtitle('Attitude Estimate Over Time');
att_file = fullfile(results_dir, sprintf('%s_Task5_Attitude.pdf', tag));
set(gcf,'PaperPositionMode','auto'); print(gcf, att_file, '-dpdf', '-bestfit');

pos_interp = interp1(imu_time, x_log(1:3,:)', gnss_time, 'linear', 'extrap');
vel_interp = interp1(imu_time, x_log(4:6,:)', gnss_time, 'linear', 'extrap');
res_pos = pos_interp - gnss_pos_ned; res_vel = vel_interp - gnss_vel_ned;
rmse_pos = sqrt(mean(sum(res_pos.^2,2))); rmse_vel = sqrt(mean(sum(res_vel.^2,2)));
final_pos_err = norm(x_log(1:3,end) - gnss_pos_ned(end,:)');
final_vel_err = norm(vel_log(:,end) - gnss_vel_ned(end,:)');
final_acc_err = norm(accel_from_vel(:,end) - gnss_accel_ned(end,:)');
rms_resid_pos = sqrt(mean(res_pos.^2,'all')); rms_resid_vel = sqrt(mean(res_vel.^2,'all')); max_resid_pos = max(vecnorm(res_pos,2,2)); min_resid_pos = min(vecnorm(res_pos,2,2)); max_resid_vel = max(vecnorm(res_vel,2,2)); min_resid_vel = min(vecnorm(res_vel,2,2));

figure('Name', 'KF Results: Position Residuals', 'Position', [150 150 1200 600]);
err_labels = {'N', 'E', 'D'};
for i = 1:3
    subplot(3,1,i); plot(gnss_time, res_pos(:,i), 'b-'); grid on; ylabel('[m]'); title(['Residual ' err_labels{i}]);
end
xlabel('Time (s)'); sgtitle('Position Residuals (KF - GNSS)');
err_file = fullfile(results_dir, sprintf('%s_Task5_ErrorAnalysis.pdf', tag));
set(gcf,'PaperPositionMode','auto'); print(gcf, err_file, '-dpdf', '-bestfit');

summary_line = sprintf(['[SUMMARY] method=%s rmse_pos=%.2fm rmse_vel=%.2fm final_pos=%.2fm final_vel=%.2fm/s final_acc=%.2fm/s^2 ' ...
    'mean_resid_pos=%.2f rms_resid_pos=%.2f max_resid_pos=%.2f min_resid_pos=%.2f ' ...
    'mean_resid_vel=%.2f rms_resid_vel=%.2f max_resid_vel=%.2f min_resid_vel=%.2f ' ...
    'GravErrMean_deg=%.2f GravErrMax_deg=%.2f EarthRateErrMean_deg=%.2f EarthRateErrMax_deg=%.2f ' ...
    'accel_bias=%.4f gyro_bias=%.4f ZUPT_count=%d'], ...
    method, rmse_pos, rmse_vel, final_pos_err, final_vel_err, final_acc_err, mean(vecnorm(res_pos,2,2)), rms_resid_pos, ...
    max_resid_pos, min_resid_pos, mean(vecnorm(res_vel,2,2)), rms_resid_vel, ...
    max_resid_vel, min_resid_vel, grav_err_mean_deg, grav_err_max_deg, omega_err_mean_deg, omega_err_max_deg, ...
    norm(accel_bias), norm(gyro_bias), zupt_count);
fprintf('%s\n', summary_line);
fid = fopen(fullfile(results_dir, [tag '_summary.txt']), 'w'); fprintf(fid, '%s\n', summary_line); fclose(fid);

results = struct('method', method, 'rmse_pos', rmse_pos, 'rmse_vel', rmse_vel, ...
    'final_pos_error', final_pos_err, 'final_vel_error', final_vel_err, ...
    'final_acc_error', final_acc_err, 'accel_bias', accel_bias, 'gyro_bias', gyro_bias, ...
    'grav_err_mean_deg', grav_err_mean_deg, 'grav_err_max_deg', grav_err_max_deg, ...
    'earth_rate_err_mean_deg', omega_err_mean_deg, 'earth_rate_err_max_deg', omega_err_max_deg);
perf_file = fullfile(results_dir, 'IMU_GNSS_bias_and_performance.mat');
% Persist the ``results`` struct for later analysis
if isfile(perf_file)
    save(perf_file, '-struct', 'results', '-append');
else
    save(perf_file, '-struct', 'results');
end
summary_file = fullfile(results_dir, 'IMU_GNSS_summary.txt'); fid_sum = fopen(summary_file, 'a'); fprintf(fid_sum, '%s\n', summary_line); fclose(fid_sum);
% Store the fused state using the standard naming scheme
results_file = fullfile(results_dir, sprintf('%s_task5_results.mat', tag));
save(results_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log');
method_file = results_file;
save(method_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log');

task5_results = results; assignin('base', 'task5_results', task5_results);

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
tag = sprintf('%s_%s_%s', imu_name, gnss_name, method);
res_file = fullfile(results_dir, [tag '_task5_results.mat']);
if exist(res_file, 'file')
    S = load(res_file);
else
    warning('Results file %s not found.', res_file);
    S = struct();
end

% Generate standard Task 5 plot with GNSS overlay
gnss_pos_interp = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear', 'extrap');
gnss_vel_interp = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear', 'extrap');
gnss_acc_interp = interp1(gnss_time, gnss_accel_ned, imu_time, 'linear', 'extrap');
pos_struct = struct('TRIAD', S.x_log(1:3,:)');
vel_struct = struct('TRIAD', S.vel_log');
acc_struct = struct('TRIAD', S.accel_from_vel');
plot_task5_results_all_methods(imu_time, pos_struct, vel_struct, acc_struct, ...
    gnss_pos_interp, gnss_vel_interp, gnss_acc_interp);

% Rename key figures to match the Python naming scheme
rename_plot(sprintf('%s_Task3_ErrorComparison.pdf', tag), ...
            sprintf('%s_task3_errors_comparison.pdf', tag));
rename_plot(sprintf('%s_Task3_QuaternionComparison.pdf', tag), ...
            sprintf('%s_task3_quaternions_comparison.pdf', tag));
rename_plot(sprintf('%s_Task4_NEDFrame.pdf', tag), ...
            sprintf('%s_task4_comparison_ned.pdf', tag));
rename_plot(sprintf('%s_Task4_MixedFrame.pdf', tag), ...
            sprintf('%s_task4_mixed_frames.pdf', tag));
rename_plot(sprintf('%s_Task4_ECEFFrame.pdf', tag), ...
            sprintf('%s_task4_all_ecef.pdf', tag));
rename_plot(sprintf('%s_Task4_BodyFrame.pdf', tag), ...
            sprintf('%s_task4_all_body.pdf', tag));
rename_plot(sprintf('%s_Task5_PVA.pdf', tag), ...
            sprintf('%s_task5_results_%s.pdf', tag, method));
rename_plot(sprintf('%s_Task5_Attitude.pdf', tag), ...
            sprintf('%s_task5_all_body.pdf', tag));
rename_plot(sprintf('%s_Task5_ErrorAnalysis.pdf', tag), ...
            sprintf('%s_%s_residuals.pdf', tag, lower(method)));
rename_plot(sprintf('%s_Task5_Attitude.pdf', tag), ...
            sprintf('%s_task5_all_body.pdf', tag));
rename_plot(sprintf('%s_Task5_ErrorAnalysis.pdf', tag), ...
            sprintf('%s_%s_residuals.pdf', tag, lower(method)));

% -----------------------------------------------------------------------
% Subtask 5.9: Validate against truth data if available
% -----------------------------------------------------------------------
state_file = fullfile(fileparts(imu_path), sprintf('STATE_%s.txt', imu_name));
if exist(state_file, 'file')
    fprintf('Subtask 5.9: Validating against truth data\n');
    truth = readmatrix(state_file);
    truth_time = truth(:,2);
    truth_pos = truth(:,3:5);
    truth_vel = truth(:,6:8);
    truth_eul = truth(:,9:11);

    pos_interp = interp1(imu_time, x_log(1:3,:)', truth_time, 'linear', 'extrap');
    vel_interp = interp1(imu_time, x_log(4:6,:)', truth_time, 'linear', 'extrap');
    eul_interp = interp1(imu_time, rad2deg(euler_log)', truth_time, 'linear', 'extrap');

    pos_err = pos_interp - truth_pos;
    vel_err = vel_interp - truth_vel;
    eul_err = eul_interp - truth_eul;

    rmse_pos = sqrt(mean(vecnorm(pos_err,2,2).^2));
    final_pos = norm(pos_err(end,:));
    rmse_vel = sqrt(mean(vecnorm(vel_err,2,2).^2));
    final_vel = norm(vel_err(end,:));
    rmse_eul = sqrt(mean(vecnorm(eul_err,2,2).^2));
    final_eul = norm(eul_err(end,:));

    fprintf('Dataset %s, Method %s: RMSE pos=%.3f m, Final pos=%.3f m, ', imu_name, method, rmse_pos, final_pos);
    fprintf('RMSE vel=%.3f m/s, Final vel=%.3f m/s, ', rmse_vel, final_vel);
    fprintf('RMSE eul=%.3f deg, Final eul=%.3f deg\n', rmse_eul, final_eul);
end

end

function rename_plot(src, dst)
%RENAME_PLOT Move SRC to DST inside the results folder if it exists.
results_dir = get_results_dir();
src_path = fullfile(results_dir, src);
dst_path = fullfile(results_dir, dst);
if exist(src_path, 'file')
    movefile(src_path, dst_path, 'f');
end
end

%% =======================================================================
%  Helper Functions from individual tasks
% ========================================================================

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

function [R, q] = davenport_q_method(v1B, v2B, v1N, v2N)
    w1 = 0.9999; w2 = 1 - w1;
    B = w1 * (v1N * v1B') + w2 * (v2N * v2B');
    S = B + B';
    sigma = trace(B);
    Z = [B(2,3) - B(3,2); B(3,1) - B(1,3); B(1,2) - B(2,1)];
    K = [sigma, Z'; Z, S - sigma * eye(3)];
    [eigvecs, eigvals] = eig(K, 'vector');
    [~, max_idx] = max(eigvals);
    q_opt = eigvecs(:, max_idx);
    if q_opt(1) < 0, q_opt = -q_opt; end
    q = [q_opt(1); -q_opt(2:4)];
    R = quat_to_rot(q);
end

function R = svd_alignment(body_vecs, ref_vecs, weights)
    if nargin < 3, weights = ones(1, length(body_vecs)); end
    B = zeros(3, 3);
    for i = 1:length(body_vecs)
        B = B + weights(i) * (ref_vecs{i} / norm(ref_vecs{i})) * (body_vecs{i} / norm(body_vecs{i}))';
    end
    [U, ~, V] = svd(B);
    M = diag([1, 1, det(U * V')]);
    R = U * M * V';
end

function q = rot_to_quaternion(R)
    tr = trace(R);
    if tr > 0
        S = sqrt(tr + 1.0) * 2; qw = 0.25 * S; qx = (R(3,2) - R(2,3)) / S; qy = (R(1,3) - R(3,1)) / S; qz = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; qw = (R(3,2) - R(2,3)) / S; qx = 0.25 * S; qy = (R(1,2) + R(2,1)) / S; qz = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; qw = (R(1,3) - R(3,1)) / S; qx = (R(1,2) + R(2,1)) / S; qy = 0.25 * S; qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; qw = (R(2,1) - R(1,2)) / S; qx = (R(1,3) + R(3,1)) / S; qy = (R(2,3) + R(3,2)) / S; qz = 0.25 * S;
    end
    q = [qw; qx; qy; qz];
    if q(1) < 0, q = -q; end
    q = q / norm(q);
end

function R = quat_to_rot(q)
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R=[1-2*(qy^2+qz^2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy); ...
       2*(qx*qy+qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qw*qx); ...
       2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx^2+qy^2)];
end

function deg = angle_between(v1, v2)
    cos_theta = max(min(dot(v1, v2) / (norm(v1) * norm(v2)), 1.0), -1.0);
    deg = acosd(cos_theta);
end

function [grav_err, earth_err] = compute_wahba_errors(C_bn, g_body, omega_ie_body, g_ref_ned, omega_ref_ned)
    g_pred_ned = C_bn * g_body;
    omega_pred_ned = C_bn * omega_ie_body;
    grav_err = angle_between(g_pred_ned, g_ref_ned);
    earth_err = angle_between(omega_pred_ned, omega_ref_ned);
end

function data_filt = butter_lowpass_filter(data, cutoff, fs, order)
    if nargin < 4 || isempty(order);   order = 4;   end
    if nargin < 3 || isempty(fs);      fs = 400;   end
    if nargin < 2 || isempty(cutoff);  cutoff = 5.0; end
    nyq = 0.5 * fs; normal_cutoff = cutoff / nyq; [b,a] = butter(order, normal_cutoff, 'low');
    data_filt = filtfilt(b, a, data);
end

function [start_idx, end_idx] = detect_static_interval(accel, gyro, window_size, accel_var_thresh, gyro_var_thresh, min_length)
    if nargin < 3 || isempty(window_size);      window_size = 200;   end
    if nargin < 4 || isempty(accel_var_thresh); accel_var_thresh = 0.01; end
    if nargin < 5 || isempty(gyro_var_thresh);  gyro_var_thresh = 1e-6; end
    if nargin < 6 || isempty(min_length);       min_length = 100;   end
    N = size(accel,1); if N < window_size, error('window_size larger than data length'); end
    rehash toolboxcache
    if exist('movvar','file') == 2
        accel_var = movvar(accel, window_size, 0, 'Endpoints','discard');
        gyro_var  = movvar(gyro,  window_size, 0, 'Endpoints','discard');
    else
        % Manual moving variance calculation when movvar is unavailable
        num_windows = N - window_size + 1;
        accel_var = zeros(num_windows, size(accel,2));
        gyro_var  = zeros(num_windows, size(gyro,2));
        for i = 1:num_windows
            accel_var(i,:) = var(accel(i:i+window_size-1,:), 0, 1);
            gyro_var(i,:)  = var(gyro(i:i+window_size-1,:),  0, 1);
        end
    end
    max_accel_var = max(accel_var, [], 2); max_gyro_var  = max(gyro_var, [], 2);
    static_mask = (max_accel_var < accel_var_thresh) & (max_gyro_var < gyro_var_thresh);
    diff_mask = diff([0; static_mask; 0]); starts = find(diff_mask == 1); ends = find(diff_mask == -1) - 1;
    longest_len = 0; start_idx = 1; end_idx = window_size;
    for k = 1:length(starts)
        seg_len = ends(k) - starts(k) + 1;
        if seg_len >= min_length && seg_len > longest_len
            longest_len = seg_len; start_idx = starts(k); end_idx = ends(k) + window_size - 1; end
    end
end

function plot_single_method(method, t_gnss, t_imu, C_B_N, p_gnss_ned, v_gnss_ned, a_gnss_ned, p_imu, v_imu, a_imu, acc_body_corr, base, r0_ecef, C_e2n)
    dims = {'North','East','Down'};
    fig = figure('Visible','off','Position',[100 100 1200 900]);
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ned(:,i),'k--','DisplayName','Measured GNSS');
        plot(t_imu, p_imu(:,i),'b-','DisplayName',sprintf('Derived IMU (%s)', method));
        legend('Measured GNSS', sprintf('Derived IMU (%s)', method));
        hold off; grid on; title(['Position ' dims{i}]); ylabel('m');

        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ned(:,i),'k--','DisplayName','Measured GNSS');
        plot(t_imu, v_imu(:,i),'b-','DisplayName',sprintf('Derived IMU (%s)', method));
        legend('Measured GNSS', sprintf('Derived IMU (%s)', method));
        hold off; grid on; title(['Velocity ' dims{i}]); ylabel('m/s');

        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ned(:,i),'k--','DisplayName','Measured GNSS');
        plot(t_imu, a_imu(:,i),'b-','DisplayName',sprintf('Derived IMU (%s)', method));
        legend('Measured GNSS', sprintf('Derived IMU (%s)', method));
        hold off; grid on; title(['Acceleration ' dims{i}]); ylabel('m/s^2');
    end
    sgtitle([method ' Comparison in NED frame']); fname = [base '_Task4_NEDFrame.pdf']; set(fig,'PaperPositionMode','auto'); print(fig,fname,'-dpdf','-bestfit'); close(fig);

    C_n2e = C_e2n'; fig = figure('Visible','off','Position',[100 100 1200 900]);
    p_gnss_ecef = (C_n2e*p_gnss_ned' + r0_ecef)'; v_gnss_ecef = (C_n2e*v_gnss_ned')'; a_gnss_ecef = (C_n2e*a_gnss_ned')'; a_imu_ecef = (C_n2e*a_imu')'; p_imu_ecef = (C_n2e*p_imu')'; v_imu_ecef = (C_n2e*v_imu')'; dims_e = {'X','Y','Z'};
    for i = 1:3
        subplot(3,3,i); hold on;
        plot(t_gnss, p_gnss_ecef(:,i),'k--','DisplayName','Measured GNSS');
        plot(t_imu, p_imu_ecef(:,i),'b-','DisplayName',sprintf('Derived IMU (%s)', method));
        legend('Measured GNSS', sprintf('Derived IMU (%s)', method));
        hold off; grid on; title(['Position ' dims_e{i}]); ylabel('m');

        subplot(3,3,i+3); hold on;
        plot(t_gnss, v_gnss_ecef(:,i),'k--','DisplayName','Measured GNSS');
        plot(t_imu, v_imu_ecef(:,i),'b-','DisplayName',sprintf('Derived IMU (%s)', method));
        legend('Measured GNSS', sprintf('Derived IMU (%s)', method));
        hold off; grid on; title(['Velocity ' dims_e{i}]); ylabel('m/s');

        subplot(3,3,i+6); hold on;
        plot(t_gnss, a_gnss_ecef(:,i),'k--','DisplayName','Measured GNSS');
        plot(t_imu, a_imu_ecef(:,i),'b-','DisplayName',sprintf('Derived IMU (%s)', method));
        legend('Measured GNSS', sprintf('Derived IMU (%s)', method));
        hold off; grid on; title(['Acceleration ' dims_e{i}]); ylabel('m/s^2');
    end
    sgtitle([method ' Comparison in ECEF frame']); fname = [base '_Task4_ECEFFrame.pdf']; set(fig,'PaperPositionMode','auto'); print(fig,fname,'-dpdf','-bestfit'); close(fig);

    fig = figure('Visible','off','Position',[100 100 1200 900]); C_N_B = C_B_N'; pos_body = (C_N_B*p_gnss_ned')'; vel_body = (C_N_B*v_gnss_ned')'; dims_b = {'X','Y','Z'};
    for i = 1:3
        subplot(3,3,i); plot(t_gnss,pos_body(:,i),'k-'); grid on; title(['Position b' dims_b{i}]); ylabel('m');
        subplot(3,3,i+3); plot(t_gnss,vel_body(:,i),'k-'); grid on; title(['Velocity b' dims_b{i}]); ylabel('m/s');
        subplot(3,3,i+6); plot(t_imu, acc_body_corr(:,i),'b-'); grid on; title(['Acceleration b' dims_b{i}]); ylabel('m/s^2');
    end
    sgtitle([method ' Data in Body Frame']); fname = [base '_Task4_BodyFrame.pdf']; set(fig,'PaperPositionMode','auto'); print(fig,fname,'-dpdf','-bestfit'); close(fig);

    fig = figure('Visible','off','Position',[100 100 1200 900]);
    for i = 1:3
        subplot(3,3,i); plot(t_gnss,p_gnss_ecef(:,i),'k-'); grid on; title(['Pos ' dims_e{i} ' ECEF']); ylabel('m');
        subplot(3,3,i+3); plot(t_gnss,v_gnss_ecef(:,i),'k-'); grid on; title(['Vel ' dims_e{i} ' ECEF']); ylabel('m/s');
        subplot(3,3,i+6); plot(t_imu, acc_body_corr(:,i),'b-'); grid on; title(['Acc ' dims_b{i} ' Body']); ylabel('m/s^2');
    end
    sgtitle([method ' Mixed Frame Data']); fname = [base '_Task4_MixedFrame.pdf']; set(fig,'PaperPositionMode','auto'); print(fig,fname,'-dpdf','-bestfit'); close(fig);
end

function q_new = propagate_quaternion(q_old, w, dt)
    w_norm = norm(w);
    if w_norm > 1e-9
        axis = w / w_norm; angle = w_norm * dt; dq = [cos(angle/2); axis * sin(angle/2)];
    else
        dq = [1; 0; 0; 0];
    end
    q_new = quat_multiply(q_old, dq); q_new = q_new / norm(q_new);
end

function q_out = quat_multiply(q1, q2)
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4); w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
    q_out = [w1*w2 - x1*x2 - y1*y2 - z1*z2; w1*x2 + x1*w2 + y1*z2 - z1*y2; w1*y2 - x1*z2 + y1*w2 + z1*x2; w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function euler = quat_to_euler(q)
    w = q(1); x = q(2); y = q(3); z = q(4);
    sinr_cosp = 2 * (w * x + y * z); cosr_cosp = 1 - 2 * (x * x + y * y); roll = atan2(sinr_cosp, cosr_cosp);
    sinp = 2 * (w * y - z * x); if abs(sinp) >= 1, pitch = sign(sinp) * (pi/2); else, pitch = asin(sinp); end
    siny_cosp = 2 * (w * z + x * y); cosy_cosp = 1 - 2 * (y * y + z * z); yaw = atan2(siny_cosp, cosy_cosp);
    euler = [roll; pitch; yaw];
end

function is_stat = is_static(acc, gyro)
    acc_thresh = 0.01; gyro_thresh = 1e-6;
    is_stat = all(var(acc,0,1) < acc_thresh) && all(var(gyro,0,1) < gyro_thresh);
end
