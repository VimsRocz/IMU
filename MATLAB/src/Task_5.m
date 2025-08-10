function result = Task_5(imu_path, gnss_path, method, gnss_pos_ned, varargin)
%TASK_5  Run 15-state EKF using IMU & GNSS NED positions
%   Expects Task 1 outputs saved in the results directory for gravity
%   initialization.
%
%   The accelerometer scale factor estimated in Task 4 is applied once to
%   bias-corrected accelerations.  When no scale factor is supplied or
%   available, a neutral factor of 1.0 is used.
%
%   result = Task_5(imu_path, gnss_path, method, gnss_pos_ned)
%   Optional name/value arguments mirror the Python Kalman filter defaults:
%       'accel_noise'      - process noise for acceleration             [m/s^2]  (0.1)
%       'vel_proc_noise'   - extra velocity process noise               [m/s^2]  (0.0)
%       'pos_proc_noise'   - position process noise                     [m]      (0.0)
%       'pos_meas_noise'   - GNSS position measurement noise            [m]      (1.0)
%       'vel_meas_noise'   - GNSS velocity measurement noise            [m/s]    (1.0)
%       'accel_bias_noise' - accelerometer bias random walk             [m/s^2]  (1e-5)
%       'gyro_bias_noise'  - gyroscope bias random walk                 [rad/s]  (1e-5)
%       'vel_q_scale'      - scale for Q(4:6,4:6) velocity process noise [-]      (10.0)
%       'vel_r'            - R(4:6,4:6) velocity measurement variance   [m^2/s^2] (0.25)
%       'scale_factor'     - accelerometer scale factor                 [-]      (required)
%       'trace_first_n'    - save state/covariance for first N steps    [samples] (0)

paths = project_paths();
results_dir = paths.matlab_results;
addpath(fullfile(paths.root,'MATLAB','lib'));
try
    cfg = evalin('caller','cfg');
catch
    error('cfg not found in caller workspace');
end
visibleFlag = 'off';
try
    if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
        visibleFlag = 'on';
    end
catch
end
    if nargin < 1 || isempty(imu_path)
        error('IMU path not specified');
    end
    if nargin < 2 || isempty(gnss_path)
        error('GNSS path not specified');
    end
    if nargin < 3 || isempty(method)
        method = 'TRIAD';
    end

    % Optional noise parameters
    p = inputParser;
    addParameter(p, 'accel_noise', 0.1);       % [m/s^2]
    addParameter(p, 'vel_proc_noise', 0.0);    % [m/s^2]
    addParameter(p, 'pos_proc_noise', 0.0);    % [m]
    addParameter(p, 'pos_meas_noise', 1.0);    % [m]
    addParameter(p, 'vel_meas_noise', 1.0);    % [m/s]
    addParameter(p, 'accel_bias_noise', 1e-5); % [m/s^2]
    addParameter(p, 'gyro_bias_noise', 1e-5);  % [rad/s]
    addParameter(p, 'vel_q_scale', 10.0);      % [-]
    addParameter(p, 'vel_r', 0.25);            % [m^2/s^2]
    addParameter(p, 'scale_factor', []);       % [-]
    addParameter(p, 'trace_first_n', 0);       % [samples]
    parse(p, varargin{:});
    accel_noise     = p.Results.accel_noise;
    vel_proc_noise  = p.Results.vel_proc_noise;
    pos_proc_noise  = p.Results.pos_proc_noise;
    pos_meas_noise  = p.Results.pos_meas_noise;
    vel_meas_noise  = p.Results.vel_meas_noise;
    accel_bias_noise = p.Results.accel_bias_noise;
    gyro_bias_noise  = p.Results.gyro_bias_noise;
    vel_q_scale     = p.Results.vel_q_scale;
    vel_r           = p.Results.vel_r;
    scale_factor    = p.Results.scale_factor;
    trace_first_n   = p.Results.trace_first_n;


    if ~isfile(gnss_path)
        error('Task_5:GNSSFileNotFound', ...
              'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
              gnss_path);
    end
    if ~isfile(imu_path)
        error('Task_5:IMUFileNotFound', ...
              'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
              imu_path);
    end
    [~, imu_name, ~] = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
    % Consistent IDs: pair_tag without method; run_id with method
    pair_tag = sprintf('%s_%s', imu_name, gnss_name);
    run_id = sprintf('%s_%s_%s', imu_name, gnss_name, method);

    if isempty(method)
        log_tag = '';
    else
        log_tag = [' (' method ')'];
    end
    fprintf('\nTask 5: Sensor Fusion with Kalman Filter\n');
    fprintf('Subtask 5.1: Configuring logging.\n');

    % Load attitude estimate from Task 3 results
    results_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
    if evalin('base','exist(''task3_results'',''var'')')
        task3_results = evalin('base','task3_results');
    else
        data = load(results_file);
        task3_results = data.task3_results;
    end
    % Support both Task_3 schema variants
    if isfield(task3_results, 'Rbn') && isfield(task3_results.Rbn, method)
        C_B_N = task3_results.Rbn.(method);
    elseif isfield(task3_results, method) && isfield(task3_results.(method), 'R')
        C_B_N = task3_results.(method).R;
    else
        error('Method %s not found in task3_results (checked Rbn and legacy fields).', method);
    end
    if strcmpi(method,'TRIAD')
        assignin('base','C_B_N_ref', C_B_N);
    end

    fprintf('Subtask 5.3: Loading GNSS and IMU data.\n');
    % Load GNSS data to obtain time and velocity
    gnss_tbl = readtable(gnss_path);
    gnss_time = zero_base_time(gnss_tbl.Posix_Time);
    pos_cols = {'X_ECEF_m','Y_ECEF_m','Z_ECEF_m'};
    gnss_pos_ecef = gnss_tbl{:, pos_cols};
    vx = gnss_tbl.VX_ECEF_mps;
    vy = gnss_tbl.VY_ECEF_mps;
    vz = gnss_tbl.VZ_ECEF_mps;
    gnss_vel_ecef = [vx vy vz];
    first_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
    ref_r0 = gnss_pos_ecef(first_idx, :)';
    [lat_deg, lon_deg, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
    C_ECEF_to_NED = R_ecef_to_ned(deg2rad(lat_deg), deg2rad(lon_deg));
    I = C_ECEF_to_NED * C_ECEF_to_NED';
    err = norm(I - eye(3), 'fro');
    assert(err < 1e-9, 'R_ecef_to_ned not orthonormal (||I-eye||=%.3e)', err);
    omega_E = constants.EARTH_RATE;
    omega_ie_NED = omega_E * [cosd(lat_deg); 0; -sind(lat_deg)];
    % Make reference parameters available for later plotting scripts
    assignin('base','ref_lat', deg2rad(lat_deg));
    assignin('base','ref_lon', deg2rad(lon_deg));
    assignin('base','ref_r0',  ref_r0);
    % Convert GNSS measurements from ECEF to NED to guarantee a common frame
    gnss_pos_ned_calc = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
    gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';
    if nargin >= 4 && ~isempty(gnss_pos_ned)
        diff_pos = max(abs(gnss_pos_ned(:) - gnss_pos_ned_calc(:)));
        if diff_pos > 1e-3
            warning('Provided gnss\_pos\_ned differs from computed NED positions (max %.3f m); using computed values.', diff_pos);
            gnss_pos_ned = gnss_pos_ned_calc;
        end
    else
        gnss_pos_ned = gnss_pos_ned_calc;
    end
    dt_gnss = diff(gnss_time);
    gnss_accel_ned  = [zeros(1,3); diff(gnss_vel_ned) ./ dt_gnss];
    gnss_accel_ecef = [zeros(1,3); diff(gnss_vel_ecef) ./ dt_gnss];

    % Load IMU data
    imu_raw = readmatrix(imu_path);
    dt_imu = mean(diff(imu_raw(1:100,2)));
    if dt_imu <= 0 || isnan(dt_imu)
        dt_imu = 1/400;
    end
    imu_time = (0:size(imu_raw,1)-1)' * dt_imu;
    gyro_body_raw = imu_raw(:,3:5) / dt_imu;
    acc_body_raw = imu_raw(:,6:8) / dt_imu;

    % Detect a static interval for ZUPT handling using the same helper as
    % Python. The raw measurements are low-pass filtered prior to
    % variance-based detection to avoid spurious motion being classified as
    % static.
    fs = 1/dt_imu;
    gyro_filt = low_pass_filter(gyro_body_raw, 10, fs);
    acc_filt  = low_pass_filter(acc_body_raw, 10, fs);
    [static_start, static_end] = detect_static_interval(acc_filt, gyro_filt, 80, 0.05, 0.005);

    % Load biases estimated in Task 2
    task2_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', ...
        imu_name, gnss_name, method));
    if isfile(task2_file)
        S2 = load(task2_file);
        bd = S2.body_data;
        accel_bias = bd.accel_bias(:);
        gyro_bias = bd.gyro_bias(:);
        if isfield(bd, 'g_body');         g_body = bd.g_body(:);         else; g_body = zeros(3,1); end
        if isfield(bd, 'omega_ie_body');  omega_ie_body = bd.omega_ie_body(:); else; omega_ie_body = zeros(3,1); end
        if isfield(bd, 'accel_scale'); accel_scale = bd.accel_scale; else; accel_scale = 1.0; end
    else
        warning('Task 2 results not found, estimating biases from first samples');
        N_static = min(4000, size(acc_body_raw,1));
        accel_bias = mean(acc_body_raw(1:N_static,:),1)';
        gyro_bias = mean(gyro_body_raw(1:N_static,:),1)';
        g_body = -mean(acc_body_raw(1:N_static,:),1)';
        omega_ie_body = mean(gyro_body_raw(1:N_static,:),1)';
        accel_scale = 1.0;
    end
    % Use accelerometer scale factor from Task 2 when not supplied
    if isempty(scale_factor)
        if ~isempty(accel_scale) && isfinite(accel_scale)
            scale_factor = accel_scale;
        else
            error('Task 5: accel_scale missing from Task 2/4 output');
        end
    end
    % Re-compute accelerometer bias using full static interval and rotated gravity
    try
        static_range = static_start:static_end-1;
        mean_acc_body = mean(acc_body_raw(static_range,:),1)';
        C_n_b = C_B_N';
        g_ned_bias = [0; 0; 9.794];
        accel_bias = mean_acc_body + C_n_b * g_ned_bias;
        fprintf('Computed accel bias in Task 5: [%f, %f, %f]\n', accel_bias);
    catch
        warning('Task 5: accel bias recomputation failed; using Task 2 bias');
    end
    fprintf('Method %s: Scale factor: %.4f\n', method, scale_factor);

    % Apply bias correction to IMU data
    gyro_body_raw = gyro_body_raw - gyro_bias';
    acc_body_raw  = scale_factor * (acc_body_raw - accel_bias');



%% ========================================================================
% Subtask 5.1-5.5: Configure and Initialize 15-State Filter
% =========================================================================
fprintf('\nSubtask 5.1-5.5: Configuring and Initializing 15-State Kalman Filter.\n');
results4 = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
if nargin < 4 || isempty(gnss_pos_ned)
    if evalin('base','exist(''task4_results'',''var'')')
        gnss_pos_ned = evalin('base','task4_results.gnss_pos_ned');
    else
        if ~isfile(results4)
            error('Task_5:MissingResults', ...
                  'Task 4 must run first and save gnss_pos_ned.');
        end
        S = load(results4,'gnss_pos_ned');
        if ~isfield(S,'gnss_pos_ned')
            error('Task_5:BadMATfile', ...
                  '''gnss_pos_ned'' not found in %s', results4);
        end
        gnss_pos_ned = S.gnss_pos_ned;
    end
end
% State vector x: [pos; vel; euler; accel_bias; gyro_bias] (15x1)
init_eul = quat_to_euler(rot_to_quaternion(C_B_N));
x = zeros(15, 1);
x(1:3)  = gnss_pos_ned(1,:)';
x(4:6)  = gnss_vel_ned(1,:)';
x(7:9)  = init_eul;
x(10:12) = accel_bias(:);
x(13:15) = gyro_bias(:);
% --- EKF tuning parameters (aligned with Python defaults) ---
P = eye(15);                         % Larger initial uncertainty
P(7:9,7:9)   = eye(3) * deg2rad(5)^2; % Attitude uncertainty (5 deg)
P(10:15,10:15) = eye(6) * 1e-4;      % Bias uncertainty

% Process/measurement noise with auto-tune fallback to Python defaults
try
    [Q,R] = task5_autotune(acc_body_raw, gyro_body_raw, dt_imu);
catch
    Q = eye(15) * 1e-4;
    Q(4:6,4:6) = eye(3) * 0.1;
    Q(10:12,10:12) = eye(3) * accel_bias_noise;
    Q(13:15,13:15) = eye(3) * gyro_bias_noise;
    R = zeros(6);
    R(1:3,1:3) = eye(3) * pos_meas_noise^2;
    R(4:6,4:6) = eye(3) * 0.25;
    fprintf('Auto-tune failed; using default Q/R\n');
end
Q(4:6,4:6) = eye(3) * 0.1;
R(4:6,4:6) = eye(3) * 0.25;
fprintf('Task-5: Q_vel=0.100, R_vel=0.250 (Python parity)\n');
H = [eye(6), zeros(6,9)];

% --- Attitude Initialization ---
q_b_n = rot_to_quaternion(C_B_N); % Initial attitude quaternion

    % Gravity vector in NED frame from Task 1 initialization if available
    % Prefer the method-specific filename but fall back to a generic one
    % Task 1 results may be saved under two naming schemes. First check the
    % legacy ``Task1_init_*`` file, then fall back to the unified
    % ``*_task1_results`` produced by ``save_task_results``.
    task1_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', run_id));
    if ~isfile(task1_file)
        alt_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', pair_tag));
        if isfile(alt_file)
            task1_file = alt_file;
        else
            % Look for the newer naming convention
            alt_file = fullfile(results_dir, sprintf('%s_task1_results.mat', run_id));
            if ~isfile(alt_file)
                alt_file = fullfile(results_dir, sprintf('%s_task1_results.mat', pair_tag));
            end
            if isfile(alt_file)
                task1_file = alt_file;
            end
        end
    end

    if isfile(task1_file)
        init_data = load(task1_file);
        if isfield(init_data, 'gravity_ned')
            g_NED = init_data.gravity_ned;
        elseif isfield(init_data, 'g_NED')
            g_NED = init_data.g_NED;
        elseif isfield(init_data, 'results') && isfield(init_data.results, 'g_NED')
            g_NED = init_data.results.g_NED;
        else
            warning('Task_5:MissingField', ...
                'File %s does not contain g_NED. Using default gravity.', task1_file);
            g_NED = [0; 0; constants.GRAVITY];
        end
        fprintf('Loaded gravity from %s\n', task1_file);
    else
        warning('Task_5:MissingTask1', ...
            'Task 1 output not found; using constants.GRAVITY.');
        g_NED = [0; 0; constants.GRAVITY];
    end

    fprintf('Gravity vector applied: [%.8f %.8f %.8f]\n', g_NED);

    % -- Compute Wahba Errors using all Task 3 rotation matrices --
    methods_all = fieldnames(task3_results);
    grav_errors = zeros(1, numel(methods_all));
    omega_errors = zeros(1, numel(methods_all));
    for mi = 1:numel(methods_all)
        Rtmp = task3_results.(methods_all{mi}).R;
        [grav_errors(mi), omega_errors(mi)] = compute_wahba_errors(Rtmp, g_body, omega_ie_body, g_NED, omega_ie_NED);
    end
    grav_err_mean  = mean(grav_errors);
    grav_err_max   = max(grav_errors);
    omega_err_mean = mean(omega_errors);
    omega_err_max  = max(omega_errors);

% Trapezoidal integration state
prev_a_ned = zeros(3,1);
prev_vel = x(4:6);

% --- Pre-allocate Log Arrays ---
num_steps = size(acc_body_raw, 1);
x_log = zeros(15, num_steps);
fprintf('Task 5: x_log initialized with size %dx%d\n', size(x_log));
euler_log = zeros(3, num_steps);
zupt_log = zeros(1, num_steps);
zupt_vel_norm = nan(1, num_steps); % velocity norm after each ZUPT
acc_log = zeros(3, num_steps); % Acceleration from propagated IMU data
num_imu_samples = num_steps;
zupt_count = 0;
zupt_fail_count = 0;            % count ZUPT events not clamped to zero
vel_blow_count = 0;             % track number of velocity blow-ups
accel_std_thresh = 0.05;        % [m/s^2]
gyro_std_thresh  = 0.005;       % [rad/s]
vel_thresh       = 0.1;         % [m/s]

trace_first_n = min(trace_first_n, num_steps);
if trace_first_n > 0
    fprintf('Tracing first %d KF steps.\n', trace_first_n);
    trace_data.x = zeros(15, trace_first_n);
    trace_data.P = zeros(15,15, trace_first_n);
end
fprintf('-> 15-State filter initialized.\n');
fprintf('Subtask 5.4: Integrating IMU data for each method.\n');

%% ========================================================================
% Subtask 5.6: Kalman Filter for Sensor Fusion
% =========================================================================
fprintf('\nSubtask 5.6: Running Kalman Filter for sensor fusion for each method.\n');

% Interpolate GNSS measurements to IMU timestamps
gnss_pos_interp = zeros(num_imu_samples,3);
gnss_vel_interp = zeros(num_imu_samples,3);
for k = 1:3
    gnss_pos_interp(:,k) = interp1(gnss_time, gnss_pos_ned(:,k), imu_time, 'linear', 'extrap');
    gnss_vel_interp(:,k) = interp1(gnss_time, gnss_vel_ned(:,k), imu_time, 'linear', 'extrap');
    % Clamp out-of-bounds values to nearest GNSS sample to mirror np.interp
    gnss_pos_interp(imu_time < gnss_time(1),k) = gnss_pos_ned(1,k);
    gnss_pos_interp(imu_time > gnss_time(end),k) = gnss_pos_ned(end,k);
    gnss_vel_interp(imu_time < gnss_time(1),k) = gnss_vel_ned(1,k);
    gnss_vel_interp(imu_time > gnss_time(end),k) = gnss_vel_ned(end,k);
end
% Compare raw and interpolated GNSS data
task5_gnss_interp_ned_plot(gnss_time, gnss_pos_ned, gnss_vel_ned, imu_time, ...
    gnss_pos_interp, gnss_vel_interp, run_id, results_dir, cfg);

% --- Main Filter Loop ---
fprintf('-> Starting filter loop over %d IMU samples...\n', num_imu_samples);
for i = 1:num_imu_samples
    % Interpolate GNSS to the current IMU timestamp so the measurement
    % aligns with the state about to be propagated.  This mirrors the
    % Python helper ``interpolate_series`` used in GNSS_IMU_Fusion.py.
    gnss_pos_i = gnss_pos_interp(i,:)';
    gnss_vel_i = gnss_vel_interp(i,:)';

    if mod(i, 1e5) == 0
        fprintf('[DBG-KF] k=%d   posN=%.1f  velN=%.2f  accN=%.2f\n', ...
            i, x(1), x(4), a_ned(1));
    end
    % --- 1. State Propagation (Prediction) ---
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt_imu;
    P = F * P * F' + Q * dt_imu;

    % --- 2. Attitude Propagation ---
    corrected_gyro = gyro_body_raw(i,:)' - x(13:15);
    corrected_accel = acc_body_raw(i,:)' - x(10:12);
    current_omega_ie_b = C_B_N' * omega_ie_NED;
    w_b = corrected_gyro - current_omega_ie_b;
    % Quaternion propagation mirrors ``quat_from_rate`` in the Python
    % pipeline and keeps the attitude normalised for numerical stability.
    q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
    C_B_N = quat_to_rot(q_b_n);
    % The accelerometer measures specific force which already includes
    % gravity.  To obtain inertial acceleration we must subtract the
    % gravity vector expressed in the navigation frame.  This mirrors the
    % Python implementation in ``integration.py`` and ensures both
    % pipelines stay in sync.
    a_ned = C_B_N * corrected_accel - g_NED;
    if mod(i, 1e5) == 0
        fprintf('[DBG-KF] k=%d   posN=%.1f  velN=%.2f  accN=%.2f\n', ...
            i, x(1), x(4), a_ned(1));
    end
    if i > 1
        % Trapezoidal integration mirrors the Python fusion pipeline and
        % improves numerical stability over simple Euler steps.
        vel_new = prev_vel + 0.5 * (a_ned + prev_a_ned) * dt_imu;
        if norm(vel_new) > 500
            vel_new = prev_vel;
            pos_new = x(1:3);
            vel_blow_count = vel_blow_count + 1;
            fprintf('Velocity blow-up at k=%d; zeroed delta_v\n', i);
        else
            pos_new = x(1:3) + 0.5 * (vel_new + prev_vel) * dt_imu;
        end
    else
        vel_new = x(4:6);
        pos_new = x(1:3);
    end
    x(4:6) = vel_new;
    x(1:3) = pos_new;
    x(7:9) = quat_to_euler(q_b_n);
    acc_log(:,i) = a_ned;

    % --- 3. Measurement Update (Correction) ---
    z = [gnss_pos_i; gnss_vel_i];
    y = z - H * x;
    S = H * P * H' + R;
    K = (P * H') / S;
    x = x + K * y;
    P = (eye(15) - K * H) * P;
    % update integrator history after correction
    prev_vel = x(4:6);
    prev_a_ned = a_ned;

    % --- 5. Zero-Velocity Update (ZUPT) ---
    win_size = 80;
    acc_win = acc_body_raw(max(1,i-win_size+1):i, :);
    gyro_win = gyro_body_raw(max(1,i-win_size+1):i, :);
    acc_std = max(std(acc_win,0,1));
    gyro_std = max(std(gyro_win,0,1));
    if acc_std < accel_std_thresh && gyro_std < gyro_std_thresh && norm(x(4:6)) < vel_thresh
        zupt_count = zupt_count + 1;
        zupt_log(i) = 1;
        H_z = [zeros(3,3), eye(3), zeros(3,9)];
        R_z = eye(3) * 1e-6;
        y_z = -H_z * x;
        S_z = H_z * P * H_z' + R_z;
        K_z = (P * H_z') / S_z;
        x = x + K_z * y_z;
        P = (eye(15) - K_z * H_z) * P;
        zupt_vel_norm(i) = norm(x(4:6));
        if zupt_vel_norm(i) > vel_thresh
            zupt_fail_count = zupt_fail_count + 1;
            fprintf('ZUPT clamp failure at k=%d (norm=%.3f)\n', i, zupt_vel_norm(i));
        end
        x(4:6) = 0;
    end
    if mod(i,100000) == 0
        fprintf('ZUPT applied %d times so far\n', zupt_count);
    end

    % --- Log State and Attitude ---
    x_log(:, i) = x;
    euler_log(:, i) = quat_to_euler(q_b_n);
    if trace_first_n > 0 && i <= trace_first_n
        trace_data.x(:,i) = x;
        trace_data.P(:,:,i) = P;
    end
    if mod(i, 100000) == 0
        fprintf('Task 5: Stored state at sample %d/%d\n', i, num_imu_samples);
    end
end
fprintf('Method %s: IMU data integrated.\n', method);
fprintf('Method %s: Kalman Filter completed. ZUPTcnt=%d\n', method, zupt_count);
fprintf('Method %s: velocity blow-up events=%d\n', method, vel_blow_count);
fprintf('Method %s: ZUPT clamp failures=%d\n', method, zupt_fail_count);

%% ========================================================================
% Subtask 5.7: Handle Event at 5000s
% =========================================================================
fprintf('\nSubtask 5.7: No event handling needed as time < 5000s.\n');

%% ========================================================================
% Subtask 5.8: Plotting Results
% =========================================================================
fprintf('\nSubtask 5.8.2: Plotting results for %s.\n', method);

% Ensure array sizes match for plotting
if numel(imu_time) ~= size(x_log,2)
    N = min(numel(imu_time), size(x_log,2));
    imu_time = imu_time(1:N);
    x_log = x_log(:,1:N);
    euler_log = euler_log(:,1:N);
    zupt_log = zupt_log(1:N);
end
if numel(gnss_time) ~= size(gnss_pos_ned,1)
    N = min(numel(gnss_time), size(gnss_pos_ned,1));
    gnss_time = gnss_time(1:N);
    gnss_pos_ned = gnss_pos_ned(1:N,:);
    gnss_vel_ned = gnss_vel_ned(1:N,:);
    gnss_accel_ned = gnss_accel_ned(1:N,:);
end

% Extract velocity states and derive acceleration from them
vel_log = x_log(4:6, :);
if numel(imu_time) > 1
    dt_vec = diff(imu_time);
    accel_from_vel = [zeros(3,1), diff(vel_log,1,2) ./ dt_vec'];
else
    accel_from_vel = zeros(3, numel(imu_time));
end

% 3x3 state grid for fused output
p_n_fused = x_log(1:3,:)' ;
v_n_fused = x_log(4:6,:)' ;
a_n_fused = accel_from_vel';
plot_state_grid(imu_time, p_n_fused, v_n_fused, a_n_fused, 'NED', ...
    sprintf('%s_task5', run_id), results_dir, {'Fused'}, cfg);

% --- Combined Position, Velocity and Acceleration ---
fig = figure('Name', 'KF Results: P/V/A', 'Position', [100 100 1200 900]);
labels = {'North', 'East', 'Down'};
all_file = fullfile(results_dir, sprintf('%s_Task5_AllResults.pdf', run_id));
if exist(all_file, 'file'); delete(all_file); end
for i = 1:3
    % Position
    subplot(3,3,i); hold on;
    plot(gnss_time, gnss_pos_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Raw)');
    plot(imu_time, x_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    hold off; grid on; legend; ylabel('[m]'); title(['Position ' labels{i}]);
    fprintf('Subtask 5.8.2: Plotted %s position %s: First = %.4f, Last = %.4f\n', ...
        method, labels{i}, x_log(i,1), x_log(i,end));

    % Velocity
    subplot(3,3,i+3); hold on;
    plot(gnss_time, gnss_vel_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Raw)');
    plot(imu_time, x_log(i+3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    zupt_indices = find(zupt_log);
    if ~isempty(zupt_indices)
        plot(imu_time(zupt_indices), x_log(i+3,zupt_indices), 'ro', 'MarkerSize', 3, 'DisplayName', 'ZUPT');
    end
    hold off; grid on; legend; ylabel('[m/s]'); title(['Velocity ' labels{i}]);
    fprintf('Subtask 5.8.2: Plotted %s velocity %s: First = %.4f, Last = %.4f\n', ...
        method, labels{i}, x_log(i+3,1), x_log(i+3,end));

    % Acceleration
    subplot(3,3,i+6); hold on;
    plot(gnss_time, gnss_accel_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Derived)');
    plot(imu_time, acc_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    hold off; grid on; legend; ylabel('[m/s^2]'); title(['Acceleration ' labels{i}]);
    fprintf('Subtask 5.8.2: Plotted %s acceleration %s: First = %.4f, Last = %.4f\n', ...
        method, labels{i}, acc_log(i,1), acc_log(i,end));
end
xlabel('Time (s)');
sgtitle('Kalman Filter Results vs. GNSS');
% out_pdf = fullfile(results_dir, sprintf('%s_task5_results_%s.pdf', tag, method));
% set(fig,'PaperPositionMode','auto');
% print(fig, out_pdf, '-dpdf', '-bestfit');
% fprintf('Subtask 5.8.2: %s plot saved as ''%s''\n', method, out_pdf);
% exportgraphics(fig, all_file, 'Append', true);

% --- Plot 4: Attitude (Euler Angles) ---
figure('Name', 'KF Results: Attitude', 'Position', [200 200 1200 600]);
euler_labels = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3, 1, i);
    plot(imu_time, rad2deg(euler_log(i,:)), 'b-');
    grid on; ylabel('[deg]'); title([euler_labels{i} ' Angle']);
end
xlabel('Time (s)'); sgtitle('Attitude Estimate Over Time');
% att_file = fullfile(results_dir, sprintf('%s_Task5_Attitude.pdf', tag));
% set(gcf,'PaperPositionMode','auto');
% print(gcf, att_file, '-dpdf', '-bestfit');
% fprintf('Saved plot: %s\n', att_file);

% --- Plot 5: Velocity Magnitude After ZUPTs ---
zupt_indices = find(zupt_log);
if ~isempty(zupt_indices)
    fig_zupt = figure('Name', 'Post-ZUPT Velocity', 'Position', [300 300 800 400]);
    plot(imu_time(zupt_indices), zupt_vel_norm(zupt_indices), 'bo-');
    grid on; box on;
    xlabel('Time (s)'); ylabel('|v| after ZUPT [m/s]');
    title('Velocity magnitude following each ZUPT');
    legend('|v|');
    save_plot(fig_zupt, imu_name, gnss_name, [method '_ZUPT'], 5);
end

fprintf('Plotting all data in NED frame.\n');
plot_task5_ned_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
    gnss_time, gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, method, run_id, cfg);

fprintf('Plotting all data in ECEF frame.\n');
plot_task5_ecef_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
    gnss_time, gnss_pos_ecef, gnss_vel_ecef, gnss_accel_ecef, C_ECEF_to_NED, ref_r0, method, run_id, cfg);

fprintf('Plotting all data in body frame.\n');
plot_task5_body_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, euler_log, ...
    gnss_time, gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, method, g_NED, run_id, cfg);

state_file = fullfile(fileparts(imu_path), sprintf('STATE_%s.txt', imu_name));
if exist(state_file, 'file')
    fprintf('Plotting fused ECEF trajectory with truth overlay.\n');
    plot_task5_ecef_truth(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
        state_file, C_ECEF_to_NED, ref_r0, method, run_id, cfg);
end

%% --- End-of-run summary statistics --------------------------------------
% Interpolate filter estimates to GNSS timestamps for residual analysis
% The transpose on x_log ensures interp1 operates over rows (time). The
% result should be Nx3 matching the GNSS matrices, so avoid an extra
% transpose which previously produced a 3xN array and caused dimension
% mismatches when subtracting from gnss_pos_ned.
pos_interp = interp1(imu_time, x_log(1:3,:)', gnss_time, 'linear', 'extrap');
vel_interp = interp1(imu_time, x_log(4:6,:)', gnss_time, 'linear', 'extrap');
res_pos = pos_interp - gnss_pos_ned;
res_vel = vel_interp - gnss_vel_ned;
rmse_pos = sqrt(mean(sum(res_pos.^2,2)));
rmse_vel = sqrt(mean(sum(res_vel.^2,2)));
% Both vectors are 3x1 column vectors so avoid an extra transpose which
% previously produced a 3x3 matrix due to implicit broadcasting.
final_pos_err = norm(x_log(1:3,end) - gnss_pos_ned(end,:)');
final_vel_err = norm(vel_log(:,end) - gnss_vel_ned(end,:)');
final_vel = norm(vel_log(:,end));
final_acc_err = norm(accel_from_vel(:,end) - gnss_accel_ned(end,:)');
final_vel_mag = norm(x_log(4:6,end));
assert(final_vel_mag < 500, ...
    sprintf('KF diverged: final velocity %.2f m/s exceeds 500 m/s - check F & H matrices', final_vel_mag));
rms_resid_pos = sqrt(mean(res_pos.^2,'all'));
rms_resid_vel = sqrt(mean(res_vel.^2,'all'));
max_resid_pos = max(vecnorm(res_pos,2,2));
min_resid_pos = min(vecnorm(res_pos,2,2));
max_resid_vel = max(vecnorm(res_vel,2,2));
min_resid_vel = min(vecnorm(res_vel,2,2));

% Print a concise summary matching the Python pipeline
fprintf('Position: North=%.4f, East=%.4f, Down=%.4f\n', ...
        x_log(1,end), x_log(2,end), x_log(3,end));
fprintf('RMSE_pos: %.4f\n', rmse_pos);

% --- Plot: Position Residuals ---
figure('Name', 'KF Results: Position Residuals', 'Position', [150 150 1200 600]);
err_labels = {'N', 'E', 'D'};
for i = 1:3
    subplot(3,1,i);
    plot(gnss_time, res_pos(:,i), 'b-');
    grid on; ylabel('[m]'); title(['Residual ' err_labels{i}]);
end
xlabel('Time (s)'); sgtitle('Position Residuals (KF - GNSS)');
% err_file = fullfile(results_dir, sprintf('%s_Task5_ErrorAnalysis.pdf', tag));
% set(gcf,'PaperPositionMode','auto');
% print(gcf, err_file, '-dpdf', '-bestfit');
% fprintf('Saved plot: %s\n', err_file);
% exportgraphics(gcf, all_file, 'Append', true);
summary_line = sprintf(['[SUMMARY] method=%s imu=%s gnss=%s rmse_pos=%8.2fm ' ...
    'final_pos=%8.2fm rms_vel=%8.2fm/s final_vel=%8.2fm/s ' ...
    'rms_resid_pos=%8.2fm max_resid_pos=%8.2fm ' ...
    'rms_resid_vel=%8.2fm max_resid_vel=%8.2fm accel_bias=%.4f gyro_bias=%.4f ' ...
    'grav_err_mean=%.4f grav_err_max=%.4f omega_err_mean=%.4f omega_err_max=%.4f ' ...
    'ZUPT_count=%d'], method, imu_name, [gnss_name '.csv'], rmse_pos, ...
    final_pos_err, rmse_vel, final_vel, rms_resid_pos, max_resid_pos, ...
    rms_resid_vel, max_resid_vel, norm(accel_bias), norm(gyro_bias), grav_err_mean, grav_err_max, ...
    omega_err_mean, omega_err_max, zupt_count);
fprintf('%s\n', summary_line);
fprintf('[SUMMARY] method=%s rmse_pos=%.2f m final_pos=%.2f m ', ...
        method, rmse_pos, final_pos_err);
fprintf('rmse_vel=%.2f m/s final_vel=%.2f m/s\n', rmse_vel, final_vel);
fid = fopen(fullfile(results_dir, [run_id '_summary.txt']), 'w');
fprintf(fid, '%s\n', summary_line);
fclose(fid);

% Store summary metrics and biases for later analysis
results = struct('method', method, 'rmse_pos', rmse_pos, 'rmse_vel', rmse_vel, ...
    'final_pos_error', final_pos_err, 'final_vel_error', final_vel_err, ...
    'final_vel', final_vel, 'final_acc_error', final_acc_err, 'accel_bias', accel_bias, 'gyro_bias', gyro_bias, ...
    'grav_err_mean', grav_err_mean, 'grav_err_max', grav_err_max, ...
    'omega_err_mean', omega_err_mean, 'omega_err_max', omega_err_max, ...
    'vel_blow_events', vel_blow_count);
if trace_first_n > 0
    results.trace_first_n = trace_first_n;
end
perf_file = fullfile(results_dir, 'IMU_GNSS_bias_and_performance.mat');
% Result Logging -- store the metrics struct under the variable name
% ``results`` to stay in sync with the Python pipeline.
if isfile(perf_file)
    save(perf_file, '-append', 'results');
else
    save(perf_file, 'results');
end

summary_file = fullfile(results_dir, 'IMU_GNSS_summary.txt');
fid_sum = fopen(summary_file, 'a');
fprintf(fid_sum, '%s\n', summary_line);
fclose(fid_sum);

% Persist core results for unit tests and further analysis
% Persist IMU and GNSS time vectors for Tasks 6 and 7
time      = imu_time; %#ok<NASGU>  used by Task_6
gnss_time = gnss_time; %#ok<NASGU>
t_est = (0:size(x_log,2)-1)' * dt_imu; %#ok<NASGU>
fprintf('Saved t_est with length %d\n', length(t_est));
dt = dt_imu; %#ok<NASGU> IMU sample interval
imu_rate_hz = 1 / dt_imu; %#ok<NASGU> IMU sampling rate

% Convenience fields matching the Python pipeline
pos_ned = x_log(1:3,:)';
vel_ned = x_log(4:6,:)';
states.pos_ned_m  = x_log(1:3,:);
states.vel_ned_mps = x_log(4:6,:);
ref_lat = deg2rad(lat_deg); %#ok<NASGU>
ref_lon = deg2rad(lon_deg); %#ok<NASGU>

% Save using the same naming convention as the Python pipeline
% <IMU>_<GNSS>_<METHOD>_task5_results.mat
results_file = fullfile(results_dir, sprintf('%s_task5_results.mat', run_id));
save(results_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'gnss_pos_ecef', 'gnss_vel_ecef', 'gnss_accel_ecef', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log', 'zupt_vel_norm', ...
    'time', 'gnss_time', 'pos_ned', 'vel_ned', 'ref_lat', 'ref_lon', 'ref_r0', ...
    'states', 't_est', 'dt', 'imu_rate_hz');
% Provide compatibility with the Python pipeline and downstream tasks
% by storing the fused position under the generic ``pos`` field as well.
pos = pos_ned; %#ok<NASGU>
save(results_file, 'x_log', 'pos', '-append');
fprintf('State history (x_log) saved to %s\n', results_file);
if trace_first_n > 0
    save(results_file, 'trace_data', '-append');
end
if isfile(results_file)
    fprintf('Results saved to %s\n', results_file);
else
    warning('Missing %s', results_file);
end
try
    check = load(results_file, 'x_log');
    fprintf('Task 5: Verified x_log saved, size: %dx%d\n', size(check.x_log));
catch
    warning('Task 5: Failed to verify x_log save in %s', results_file);
end

% Export estimator time vector for compatibility with Python pipeline
time_file = fullfile(results_dir, sprintf('%s_task5_time.mat', run_id));
save(time_file, 't_est', 'dt', 'x_log');
fprintf('Task 5: Saved time vector to %s\n', time_file);

    method_struct = struct('gnss_pos_ned', gnss_pos_ned, 'gnss_vel_ned', gnss_vel_ned, ...
        'gnss_accel_ned', gnss_accel_ned, 'gnss_pos_ecef', gnss_pos_ecef, ...
        'gnss_vel_ecef', gnss_vel_ecef, 'gnss_accel_ecef', gnss_accel_ecef, ...
        'x_log', x_log, 'vel_log', vel_log, 'accel_from_vel', accel_from_vel, ...
        'euler_log', euler_log, 'zupt_log', zupt_log, 'zupt_vel_norm', zupt_vel_norm, 'time', time, ...
        'gnss_time', gnss_time, 'pos_ned', pos_ned, 'vel_ned', vel_ned, ...
        'ref_lat', ref_lat, 'ref_lon', ref_lon, 'ref_r0', ref_r0, ...
        'Q_vel', 0.1, 'R_vel', 0.25, 'zupt_count', zupt_count, 'vel_blow_count', vel_blow_count, ...
        'accel_bias', accel_bias, 'gyro_bias', gyro_bias);
    % ``method`` already stores the algorithm name (e.g. 'TRIAD'). Use it
    % directly when saving so filenames match the Python pipeline.
    save_task_results(method_struct, imu_name, gnss_name, method, 5);

    % Expose fused position for comparison plots across methods
    assignin('base', ['pos_kf_' method], x_log(1:3,:)');
    assignin('base', 't_kf', imu_time);

% Return results structure and store in base workspace
result = results;
if trace_first_n > 0
    result.trace = trace_data;
end
assignin('base', 'task5_results', result);

end % End of main function

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================
    function q_new = propagate_quaternion(q_old, w, dt)
        %PROPAGATE_QUATERNION Propagate quaternion using angular rate.
        %   Q_NEW = PROPAGATE_QUATERNION(Q_OLD, W, DT) integrates the rate
        %   vector W over DT and multiplies the result with Q_OLD.  The output
        %   quaternion is normalised to keep its unit length, matching the
        %   Python implementation.
        w_norm = norm(w);
        if w_norm > 1e-9
            axis = w / w_norm;
            angle = w_norm * dt;
            dq = [cos(angle/2); axis * sin(angle/2)];
        else
            dq = [1; 0; 0; 0];
        end
        q_new = quat_multiply(q_old, dq);
        % normalise to unit quaternion for numerical stability
        q_new = q_new / norm(q_new);
    end

    function q_out = quat_multiply(q1, q2)
        %QUAT_MULTIPLY Hamilton product of two quaternions.
        %   Q_OUT = QUAT_MULTIPLY(Q1, Q2) multiplies Q1 by Q2 using the
        %   [w x y z] convention.
        w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
        w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
        q_out = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
                 w1*x2 + x1*w2 + y1*z2 - z1*y2;
                 w1*y2 - x1*z2 + y1*w2 + z1*x2;
                 w1*z2 + x1*y2 - y1*x2 + z1*w2];
    end

    function euler = quat_to_euler(q)
        %QUAT_TO_EULER Convert quaternion to XYZ Euler angles.
        %   EULER = QUAT_TO_EULER(Q) returns [roll; pitch; yaw] in radians for
        %   the quaternion Q = [w x y z].
        w = q(1); x = q(2); y = q(3); z = q(4);
        sinr_cosp = 2 * (w * x + y * z);
        cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

        sinp = 2 * (w * y - z * x);
        if abs(sinp) >= 1
            pitch = sign(sinp) * (pi/2);
        else
            pitch = asin(sinp);
        end

        siny_cosp = 2 * (w * z + x * y);
        cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
        euler = [roll; pitch; yaw];
    end

    function R = quat_to_rot(q)
        %QUAT_TO_ROT Convert quaternion to rotation matrix.
        %   R = QUAT_TO_ROT(Q) converts Q = [w x y z] into a 3×3 rotation matrix.
        qw = q(1); qx = q(2); qy = q(3); qz = q(4);
        R = [1 - 2 * (qy^2 + qz^2), 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy);
             2 * (qx*qy + qw*qz), 1 - 2 * (qx^2 + qz^2), 2 * (qy*qz - qw*qx);
             2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 1 - 2 * (qx^2 + qy^2)];
    end

    function q = rot_to_quaternion(R)
        %ROT_TO_QUATERNION Convert rotation matrix to quaternion.
        %   Q = ROT_TO_QUATERNION(R) converts R to a [w x y z] quaternion and
        %   normalises the result with positive scalar part.
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

    function is_stat = is_static(acc, gyro, acc_thresh, gyro_thresh)
        %IS_STATIC True if IMU window std dev is below thresholds.
        %   IS_STATIC = IS_STATIC(ACC, GYRO, ACC_THRESH, GYRO_THRESH) mirrors
        %   ``utils.is_static`` using standard deviation thresholds.
        if nargin < 3, acc_thresh = 0.05; end
        if nargin < 4, gyro_thresh = 0.005; end
        is_stat = all(std(acc,0,1) < acc_thresh) && ...
                   all(std(gyro,0,1) < gyro_thresh);
    end

    function deg = angle_between(v1, v2)
        %ANGLE_BETWEEN Angle between two 3-D vectors in degrees.
        %   DEG = ANGLE_BETWEEN(V1, V2) mirrors ``init_vectors.angle_between``.
        cos_theta = max(min(dot(v1, v2) / (norm(v1) * norm(v2)), 1.0), -1.0);
        deg = acosd(cos_theta);
    end

    function [grav_err, earth_err] = compute_wahba_errors(C_bn, g_b, omega_b, g_ref, omega_ref)
        %COMPUTE_WAHBA_ERRORS Angular errors for gravity and Earth rate.
        %   [EG, EO] = COMPUTE_WAHBA_ERRORS(C_BN, G_B, OMEGA_B, G_REF, OMEGA_REF)
        %   returns the angle between measured and reference gravity vectors and
        %   between Earth rotation vectors, matching the Python helper of the
        %   same name.
        g_pred = C_bn * g_b;
        omega_pred = C_bn * omega_b;
        grav_err = angle_between(g_pred, g_ref);
        earth_err = angle_between(omega_pred, omega_ref);
    end

    function R = euler_to_rot(eul)
        %EULER_TO_ROT Convert XYZ Euler angles to Body->NED DCM.
        %   R = EULER_TO_ROT(EUL) mirrors ``utils.euler_to_rot``. EUL is a
        %   three-element vector ``[roll pitch yaw]`` in radians.
        cr = cos(eul(1)); sr = sin(eul(1));
        cp = cos(eul(2)); sp = sin(eul(2));
        cy = cos(eul(3)); sy = sin(eul(3));
        R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
             sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
             -sp,   cp*sr,            cp*cr];
    end

    function plot_task5_mixed_frame(t, pos_ned, vel_ned, acc_ned, eul_log, C_E_N, r0, g_N, run_id, method, results_dir, all_file, cfg)
        %PLOT_TASK5_MIXED_FRAME Plot ECEF position/velocity and body accel.
        %   Saves a multi-panel figure using the given METHOD and RUN_ID.
        pos_ecef = (C_E_N' * pos_ned) + r0;
        vel_ecef = C_E_N' * vel_ned;
        N = size(acc_ned,2);
        acc_body = zeros(3,N);
        for k = 1:N
            C_B_N = euler_to_rot(eul_log(:,k));
            acc_body(:,k) = C_B_N' * (acc_ned(:,k) - g_N);
        end
        fig = figure('Name','Task5 Mixed Frame','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        dims_e = {'X','Y','Z'}; dims_b = {'X','Y','Z'};
        for i = 1:3
            subplot(3,3,i);   plot(t, pos_ecef(i,:), 'b-'); grid on;
            title(['Pos ' dims_e{i} ' ECEF']); ylabel('m'); xlabel('Time [s]');

            subplot(3,3,i+3); plot(t, vel_ecef(i,:), 'b-'); grid on;
            title(['Vel ' dims_e{i} ' ECEF']); ylabel('m/s'); xlabel('Time [s]');

            subplot(3,3,i+6); plot(t, acc_body(i,:), 'b-'); grid on;
            title(['Acc ' dims_b{i} ' Body']); ylabel('m/s^2'); xlabel('Time [s]');
        end
        sgtitle([method ' Mixed Frame Data']);
        fname = fullfile(results_dir, sprintf('%s_task5_Mixed_state', run_id));
        if cfg.plots.save_pdf
            print(fig, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        if cfg.plots.save_png
            print(fig, [fname '.png'], '-dpng');
        end
        close(fig);
    end

    function plot_task5_ned_frame(t, pos_ned, vel_ned, acc_ned, gnss_t, gnss_pos, gnss_vel, gnss_acc, method, run_id, cfg)
        %PLOT_TASK5_NED_FRAME Overlay fused, GNSS, and IMU data in NED frame.
        %   t         - IMU timestamps [s]
        %   pos_ned   - fused position [3xN] (NED)
        %   vel_ned   - fused velocity [3xN] (NED)
        %   acc_ned   - IMU specific force expressed in NED [3xN]
        %   gnss_*    - GNSS measurements in NED
        axes_labels = {'N','E','D'};
        fused_color = [0.4660 0.6740 0.1880];
        imu_color   = [0.8500 0.3250 0.0980];
        gnss_color  = [0 0.4470 0.7410];

        t_row = t(:)';
        num_samples = numel(t_row);
        % Interpolate GNSS measurements to IMU time base
        gnss_pos_i = zeros(3, num_samples);
        gnss_vel_i = zeros(3, num_samples);
        gnss_acc_i = zeros(3, num_samples);
        for k = 1:3
            gnss_pos_i(k,:) = interp1(gnss_t, gnss_pos(:,k), t_row, 'linear', 'extrap');
            gnss_vel_i(k,:) = interp1(gnss_t, gnss_vel(:,k), t_row, 'linear', 'extrap');
            gnss_acc_i(k,:) = interp1(gnss_t, gnss_acc(:,k), t_row, 'linear', 'extrap');
        end

        % IMU-only dead reckoning
        vel_imu = cumtrapz(t_row, acc_ned, 2);
        pos_imu = cumtrapz(t_row, vel_imu, 2);

        % Fused acceleration derived from fused velocity
        acc_fused = [zeros(3,1), diff(vel_ned,1,2) ./ diff(t_row)];

        figure('Name','Task5 NED Frame','Position',[100 100 1200 900],'Visible','on');
        tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
        for r = 1:3
            for c = 1:3
                nexttile; hold on;
                switch c
                    case 1
                        plot(t_row, pos_ned(r,:), 'Color', fused_color, 'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, gnss_pos_i(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, pos_imu(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Pos %s [m]', axes_labels{r});
                    case 2
                        plot(t_row, vel_ned(r,:), 'Color', fused_color, 'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, gnss_vel_i(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, vel_imu(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Vel %s [m/s]', axes_labels{r});
                    case 3
                        plot(t_row, acc_fused(r,:), 'Color', fused_color, 'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, gnss_acc_i(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, acc_ned(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Acc %s [m/s^2]', axes_labels{r});
                end
                grid on; axis tight; ylabel(ylab);
                if r==1 && c==1
                    lgd = legend('Location','best'); set(lgd,'Box','on');
                end
                hold off;
            end
        end
        sgtitle('Task 5 — All Data (NED Frame)');
        xlabel('Time [s]');
        drawnow;
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_all_ned', run_id));
        if cfg.plots.save_pdf, print(gcf, [fname '.pdf'], '-dpdf', '-bestfit'); end
        if cfg.plots.save_png, print(gcf, [fname '.png'], '-dpng'); end
        savefig(gcf, [fname '.fig']);
    end

    function plot_task5_ecef_frame(t, pos_ned, vel_ned, acc_ned, gnss_t, gnss_pos, gnss_vel, gnss_acc, C_E_N, r0, method, run_id, cfg)
        %PLOT_TASK5_ECEF_FRAME Overlay fused, GNSS, and IMU data in ECEF frame.
        %   GNSS inputs are already expressed in ECEF coordinates.
        axes_labels = {'X','Y','Z'};
        fused_color = [0.4660 0.6740 0.1880];
        imu_color   = [0.8500 0.3250 0.0980];
        gnss_color  = [0 0.4470 0.7410];

        t_row = t(:)';
        num_samples = numel(t_row);
        % Interpolate GNSS data to IMU time
        gnss_pos_i = zeros(3, num_samples);
        gnss_vel_i = zeros(3, num_samples);
        gnss_acc_i = zeros(3, num_samples);
        for k = 1:3
            gnss_pos_i(k,:) = interp1(gnss_t, gnss_pos(:,k), t_row, 'linear','extrap');
            gnss_vel_i(k,:) = interp1(gnss_t, gnss_vel(:,k), t_row, 'linear','extrap');
            gnss_acc_i(k,:) = interp1(gnss_t, gnss_acc(:,k), t_row, 'linear','extrap');
        end

        % IMU dead reckoning in NED then convert to ECEF
        vel_imu_ned = cumtrapz(t_row, acc_ned, 2);
        pos_imu_ned = cumtrapz(t_row, vel_imu_ned, 2);
        pos_imu = (C_E_N' * pos_imu_ned) + r0;
        vel_imu = C_E_N' * vel_imu_ned;
        acc_imu = C_E_N' * acc_ned;

        % Fused results converted to ECEF
        pos_fused = (C_E_N' * pos_ned) + r0;
        vel_fused = C_E_N' * vel_ned;
        acc_fused = [zeros(3,1), diff(vel_fused,1,2) ./ diff(t_row)];

        figure('Name','Task5 ECEF Frame','Position',[100 100 1200 900],'Visible','on');
        tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
        for r = 1:3
            for c = 1:3
                nexttile; hold on;
                switch c
                    case 1
                        plot(t_row, pos_fused(r,:), 'Color', fused_color, 'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, gnss_pos_i(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, pos_imu(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Pos %s [m]', axes_labels{r});
                    case 2
                        plot(t_row, vel_fused(r,:), 'Color', fused_color, 'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, gnss_vel_i(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, vel_imu(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Vel %s [m/s]', axes_labels{r});
                    case 3
                        plot(t_row, acc_fused(r,:), 'Color', fused_color, 'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, gnss_acc_i(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, acc_imu(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Acc %s [m/s^2]', axes_labels{r});
                end
                grid on; axis tight; ylabel(ylab);
                if r==1 && c==1
                    lgd = legend('Location','best'); set(lgd,'Box','on');
                end
                hold off;
            end
        end
        sgtitle('Task 5 — All Data (ECEF Frame)');
        xlabel('Time [s]');
        drawnow;
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_all_ecef', run_id));
        if cfg.plots.save_pdf, print(gcf, [fname '.pdf'], '-dpdf', '-bestfit'); end
        if cfg.plots.save_png, print(gcf, [fname '.png'], '-dpng'); end
        savefig(gcf, [fname '.fig']);
    end

    function plot_task5_body_frame(t, pos_ned, vel_ned, acc_ned, eul_log, gnss_t, gnss_pos, gnss_vel, gnss_acc, method, g_N, run_id, cfg)
        %PLOT_TASK5_BODY_FRAME Overlay fused, GNSS, and IMU data in body frame.
        axes_labels = {'X','Y','Z'};
        fused_color = [0.4660 0.6740 0.1880];
        imu_color   = [0.8500 0.3250 0.0980];
        gnss_color  = [0 0.4470 0.7410];

        t_row = t(:)';
        num_samples = numel(t_row);
        % Interpolate GNSS NED data to IMU timestamps
        gnss_pos_i = zeros(3, num_samples);
        gnss_vel_i = zeros(3, num_samples);
        gnss_acc_i = zeros(3, num_samples);
        for k = 1:3
            gnss_pos_i(k,:) = interp1(gnss_t, gnss_pos(:,k), t_row, 'linear','extrap');
            gnss_vel_i(k,:) = interp1(gnss_t, gnss_vel(:,k), t_row, 'linear','extrap');
            gnss_acc_i(k,:) = interp1(gnss_t, gnss_acc(:,k), t_row, 'linear','extrap');
        end

        % IMU-only dead reckoning (NED)
        vel_imu_ned = cumtrapz(t_row, acc_ned, 2);
        pos_imu_ned = cumtrapz(t_row, vel_imu_ned, 2);

        % Preallocate body-frame arrays
        pos_body = zeros(3,num_samples);
        vel_body = zeros(3,num_samples);
        acc_body_fused = zeros(3,num_samples);
        pos_imu_body = zeros(3,num_samples);
        vel_imu_body = zeros(3,num_samples);
        acc_imu_body = zeros(3,num_samples);
        pos_gnss_body = zeros(3,num_samples);
        vel_gnss_body = zeros(3,num_samples);
        acc_gnss_body = zeros(3,num_samples);

        for k = 1:num_samples
            C_B_N = euler_to_rot(eul_log(:,k));
            pos_body(:,k) = C_B_N' * pos_ned(:,k);
            vel_body(:,k) = C_B_N' * vel_ned(:,k);
            pos_imu_body(:,k) = C_B_N' * pos_imu_ned(:,k);
            vel_imu_body(:,k) = C_B_N' * vel_imu_ned(:,k);
            pos_gnss_body(:,k) = C_B_N' * gnss_pos_i(:,k);
            vel_gnss_body(:,k) = C_B_N' * gnss_vel_i(:,k);
            acc_imu_body(:,k) = C_B_N' * (acc_ned(:,k) - g_N);
            acc_gnss_body(:,k) = C_B_N' * gnss_acc_i(:,k);
        end

        % Fused acceleration from body-frame velocity
        acc_body_fused = [zeros(3,1), diff(vel_body,1,2) ./ diff(t_row)];

        figure('Name','Task5 Body Frame','Position',[100 100 1200 900],'Visible','on');
        tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
        for r = 1:3
            for c = 1:3
                nexttile; hold on;
                switch c
                    case 1
                        plot(t_row, pos_body(r,:), 'Color', fused_color,'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, pos_gnss_body(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, pos_imu_body(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Pos %s [m]', axes_labels{r});
                    case 2
                        plot(t_row, vel_body(r,:), 'Color', fused_color,'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, vel_gnss_body(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, vel_imu_body(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Vel %s [m/s]', axes_labels{r});
                    case 3
                        plot(t_row, acc_body_fused(r,:), 'Color', fused_color,'LineWidth',1.8,'DisplayName','Fused');
                        plot(t_row, acc_gnss_body(r,:), ':', 'Color', gnss_color,'DisplayName','GNSS');
                        plot(t_row, acc_imu_body(r,:),  '-.', 'Color', imu_color,'DisplayName','IMU');
                        ylab = sprintf('Acc %s [m/s^2]', axes_labels{r});
                end
                grid on; axis tight; ylabel(ylab);
                if r==1 && c==1
                    lgd = legend('Location','best'); set(lgd,'Box','on');
                end
                hold off;
            end
        end
        sgtitle('Task 5 — All Data (Body Frame)');
        xlabel('Time [s]');
        drawnow;
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_all_body', run_id));
        if cfg.plots.save_pdf, print(gcf, [fname '.pdf'], '-dpdf', '-bestfit'); end
        if cfg.plots.save_png, print(gcf, [fname '.png'], '-dpng'); end
        savefig(gcf, [fname '.fig']);
    end

    function plot_task5_ecef_truth(t, pos_ned, vel_ned, acc_ned, state_file, C_E_N, r0, method, run_id, cfg)
        %PLOT_TASK5_ECEF_TRUTH Overlay fused output with provided truth data.
        if ~exist(state_file,'file'); return; end
        truth = readmatrix(state_file);
        t_truth = truth(:,2);
        pos_truth = truth(:,3:5);
        vel_truth = truth(:,6:8);
        dt = diff(t_truth);
        acc_truth = [zeros(1,3); diff(vel_truth)./dt];

        pos_fused = (C_E_N' * pos_ned) + r0;
        vel_fused = C_E_N' * vel_ned;
        acc_fused = C_E_N' * acc_ned;

        figure('Name','Task5 ECEF with Truth','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        labels = {'X','Y','Z'};
        for k = 1:3
            subplot(3,3,k); hold on;
            plot(t_truth, pos_truth(:,k),'m-','DisplayName','Truth');
            plot(t, pos_fused(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{k}]); legend;

            subplot(3,3,3+k); hold on;
            plot(t_truth, vel_truth(:,k),'m-','DisplayName','Truth');
            plot(t, vel_fused(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{k}]); legend;

            subplot(3,3,6+k); hold on;
            plot(t_truth, acc_truth(:,k),'m-','DisplayName','Truth');
            plot(t, acc_fused(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{k}]); legend;
        end
        sgtitle([method ' - ECEF frame with Truth']);
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_ECEF_truth', run_id));
        if cfg.plots.save_pdf
            print(gcf, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        if cfg.plots.save_png
            print(gcf, [fname '.png'], '-dpng');
        end
        close(gcf);
    end
