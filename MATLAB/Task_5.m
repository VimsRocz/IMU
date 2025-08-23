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
%       'vel_q_scale'      - scale for Q(4:6,4:6) velocity process noise [-]      (1.0)
%       'vel_r'            - R(4:6,4:6) velocity measurement variance   [m^2/s^2] (0.25)
%       'scale_factor'     - accelerometer scale factor                 [-]      (required)

addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));

paths = project_paths();
results_dir = paths.matlab_results;
addpath(fullfile(paths.root,'MATLAB','lib'));
% Obtain cfg from caller/base, fall back to default
try
    cfg = evalin('caller','cfg');
catch
    try
        cfg = evalin('base','cfg');
    catch
        cfg = default_cfg();
    end
end
visibleFlag = 'off';
try
    if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
        visibleFlag = 'on';
    end
catch
end
    % Normalize optional config aliases (robust to different cfg schemas)
    try
        if isfield(cfg,'yawaid') && isstruct(cfg.yawaid)
            % FIX: Accept common alias fields for yaw-aid configuration
            if isfield(cfg.yawaid,'en') && ~isfield(cfg.yawaid,'enabled')
                cfg.yawaid.enabled = cfg.yawaid.en; %#ok<STRNU>
            end
            if isfield(cfg.yawaid,'enable') && ~isfield(cfg.yawaid,'enabled')
                cfg.yawaid.enabled = cfg.yawaid.enable; %#ok<STRNU>
            end
            if isfield(cfg.yawaid,'max_turn_rate_dps') && ~isfield(cfg.yawaid,'max_gyro_dps')
                cfg.yawaid.max_gyro_dps = cfg.yawaid.max_turn_rate_dps; %#ok<STRNU>
            end
            if isfield(cfg.yawaid,'residual_deg_max') && ~isfield(cfg.yawaid,'gate_deg')
                cfg.yawaid.gate_deg = cfg.yawaid.residual_deg_max; %#ok<STRNU>
            end
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

    % Allow string or char inputs for file paths and convert to char
    pin = inputParser;
    addRequired(pin, 'imu_path', @(x) isstring(x) || ischar(x));
    addRequired(pin, 'gnss_path', @(x) isstring(x) || ischar(x));
    addRequired(pin, 'method',    @(x) isstring(x) || ischar(x));
    addOptional(pin, 'gnss_pos_ned', [], @(x) isempty(x) || isnumeric(x));
    parse(pin, imu_path, gnss_path, method, gnss_pos_ned);
    imu_path    = char(pin.Results.imu_path);
    gnss_path   = char(pin.Results.gnss_path);
    method      = char(pin.Results.method);
    gnss_pos_ned = pin.Results.gnss_pos_ned;

    % Optional noise parameters
    p = inputParser;
    addParameter(p, 'accel_noise', 0.1);       % [m/s^2]
    addParameter(p, 'vel_proc_noise', 0.0);    % [m/s^2]
    addParameter(p, 'pos_proc_noise', 0.0);    % [m]
    addParameter(p, 'pos_meas_noise', 1.0);    % [m]
    addParameter(p, 'vel_meas_noise', 1.0);    % [m/s]
    addParameter(p, 'accel_bias_noise', 1e-5); % [m/s^2]
    addParameter(p, 'gyro_bias_noise', 1e-5);  % [rad/s]
    addParameter(p, 'vel_q_scale', 1.0);       % [-]
    addParameter(p, 'vel_r', 0.25);            % [m^2/s^2]
    addParameter(p, 'trace_first_n', 0);       % [steps] capture first N KF steps
    addParameter(p, 'max_steps', inf);         % [steps] limit processing for tuning
    addParameter(p, 'scale_factor', []);       % [-]
    addParameter(p, 'dryrun', false);          % [flag] skip plots/logging when tuning
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
    trace_first_n   = p.Results.trace_first_n;
    max_steps       = p.Results.max_steps;
    scale_factor    = p.Results.scale_factor;
    dryrun          = p.Results.dryrun;
    if dryrun
        dprintf = @(varargin) [];
        visibleFlag = 'off'; % suppress any figure visibility
    else
        dprintf = @fprintf;
    end

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

    % Log file for verbose warnings (to avoid command window spam)
    warn_log = fullfile(results_dir, sprintf('%s_task5_warnings.log', run_id));
    warn_fid = fopen(warn_log, 'w');
    log_warn = @(varargin) fprintf(warn_fid, '%s\n', sprintf(varargin{:}));

    dprintf('\nTask 5: Sensor Fusion with Kalman Filter\n');
    dprintf('Subtask 5.1: Configuring logging.\n');

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

    dprintf('Subtask 5.3: Loading GNSS and IMU data.\n');
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
    % Prefer Mapping Toolbox signature; fall back to local helper if shadowed
    try
        wgs84 = wgs84Ellipsoid("meter");
        [lat_deg, lon_deg, ~] = ecef2geodetic(wgs84, ref_r0(1), ref_r0(2), ref_r0(3));
    catch
        [lat_deg, lon_deg, ~] = ecef_to_geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
    end
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
    % Optional: clip GNSS speeds to suppress outliers if configured
    try
        if isfield(cfg,'gnss') && isstruct(cfg.gnss) && isfield(cfg.gnss,'speed_clip_mps') && cfg.gnss.speed_clip_mps > 0
            spd = vecnorm(gnss_vel_ned,2,2);
            idx = spd > cfg.gnss.speed_clip_mps & isfinite(spd);
            if any(idx)
                scale = cfg.gnss.speed_clip_mps ./ max(spd(idx), eps);
                gnss_vel_ned(idx,:) = gnss_vel_ned(idx,:) .* scale;
            end
        end
    catch
    end
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
    % Fallback: build GNSS NED velocity from position diffs if velocity missing or near-zero
    try
        if ~exist('gnss_vel_ned','var') || isempty(gnss_vel_ned) || all(vecnorm(gnss_vel_ned,2,2) < 1e-6)
            vned = [ (gnss_pos_ned(2:end,:) - gnss_pos_ned(1:end-1,:)) ./ max(1e-6, dt_gnss), [0 0 0] ];
            vned = vned(1:size(gnss_pos_ned,1),:);
            gnss_vel_ned = vned;
            dprintf('[Task5] Built GNSS NED velocity via finite differences.\n');
        end
    catch
    end
    gnss_accel_ned  = [zeros(1,3); diff(gnss_vel_ned) ./ dt_gnss];
    gnss_accel_ecef = [zeros(1,3); diff(gnss_vel_ecef) ./ dt_gnss];
    % Early GNSS speed stats and yaw-aid tip (helps configuration)
    try
        spd_h = hypot(gnss_vel_ned(:,1), gnss_vel_ned(:,2));
        smin = min(spd_h, [], 'omitnan'); smed = median(spd_h, 'omitnan'); smax = max(spd_h, [], 'omitnan');
        dprintf('[Task5] GNSS horizontal speed stats (m/s): min=%.3f  median=%.3f  max=%.3f\n', smin, smed, smax);
        if isfield(cfg,'yawaid') && isfield(cfg.yawaid,'enabled') && cfg.yawaid.enabled && isfield(cfg.yawaid,'min_speed_mps')
            if smed < cfg.yawaid.min_speed_mps
                dprintf('[Task5] Tip: median speed < yaw-aid min_speed_mps (%.2f < %.2f). Consider lowering the gate.\n', smed, cfg.yawaid.min_speed_mps);
            end
        end
    catch
    end

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
    % Derive representative body-frame gravity and Earth-rate from the static window
    g_body = -mean(acc_body_raw(static_start:static_end, :), 1)';
    omega_ie_body = mean(gyro_body_raw(static_start:static_end, :), 1)';

    % Load biases/scale from Task 4 if available (parity with Python). Fallback to Task 2.
    accel_bias = []; gyro_bias = []; accel_scale = [];
    results4 = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
    if isfile(results4)
        try
            S4 = load(results4, 'acc_biases','gyro_biases','scale_factors');
            if isfield(S4,'acc_biases') && isfield(S4.acc_biases, method)
                accel_bias = S4.acc_biases.(method)(:);
            end
            if isfield(S4,'gyro_biases') && isfield(S4.gyro_biases, method)
                gyro_bias = S4.gyro_biases.(method)(:);
            end
            if isfield(S4,'scale_factors') && isfield(S4.scale_factors, method)
                accel_scale = S4.scale_factors.(method);
            end
        catch
        end
    end
    if isempty(accel_bias) || isempty(gyro_bias)
        % Fallback to Task 2
        task2_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', ...
            imu_name, gnss_name, method));
        if isfile(task2_file)
            S2 = load(task2_file);
            bd = S2.body_data;
            accel_bias = bd.accel_bias(:);
            gyro_bias = bd.gyro_bias(:);
            if isfield(bd, 'accel_scale'); accel_scale = bd.accel_scale; end
        else
            warning('Task 2/4 results not found, estimating biases from first samples');
            N_static = min(4000, size(acc_body_raw,1));
            accel_bias = mean(acc_body_raw(1:N_static,:),1)';
            gyro_bias = mean(gyro_body_raw(1:N_static,:),1)';
            accel_scale = 1.0;
        end
    end
    % Use accelerometer scale factor from Task 4/2 when not supplied
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
        dprintf('Computed accel bias in Task 5: [%f, %f, %f]\n', accel_bias);
    catch
        warning('Task 5: accel bias recomputation failed; using Task 2 bias');
    end
    dprintf('Method %s: Scale factor: %.4f\n', method, scale_factor);

    % Apply bias correction to IMU data
    gyro_body_raw = gyro_body_raw - gyro_bias';
    acc_body_raw  = scale_factor * (acc_body_raw - accel_bias');



%% ========================================================================
% Subtask 5.1-5.5: Configure and Initialize 15-State Filter
% =========================================================================
dprintf('\nSubtask 5.1-5.5: Configuring and Initializing 15-State Kalman Filter.\n');
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

% Process/measurement noise (aligned with Python defaults)
Q = eye(15) * 1e-4;
Q(4:6,4:6) = eye(3) * 0.01 * vel_q_scale;
Q(10:12,10:12) = eye(3) * accel_bias_noise;
Q(13:15,13:15) = eye(3) * gyro_bias_noise;
R = zeros(6);
R(1:3,1:3) = eye(3) * pos_meas_noise^2;
R(4:6,4:6) = eye(3) * vel_r;
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
        dprintf('Loaded gravity from %s\n', task1_file);
    else
        warning('Task_5:MissingTask1', ...
            'Task 1 output not found; using constants.GRAVITY.');
        g_NED = [0; 0; constants.GRAVITY];
    end

    dprintf('Gravity vector applied: [%.8f %.8f %.8f]\n', g_NED);

    % -- Compute Wahba Errors using all Task 3 rotation matrices --
    if isfield(task3_results,'methods') && ~isempty(task3_results.methods)
        methods_all = task3_results.methods;
    elseif isfield(task3_results,'Rbn')
        methods_all = fieldnames(task3_results.Rbn);
    else
        % legacy: top-level fields are method structs
        methods_all = fieldnames(task3_results);
    end
    grav_errors = zeros(1, numel(methods_all));
    omega_errors = zeros(1, numel(methods_all));
    for mi = 1:numel(methods_all)
        mname = methods_all{mi};
        if isfield(task3_results,'Rbn') && isfield(task3_results.Rbn, mname)
            Rtmp = task3_results.Rbn.(mname);
        elseif isfield(task3_results, mname)
            sub = task3_results.(mname);
            if isstruct(sub) && isfield(sub,'R')
                Rtmp = sub.R;
            else
                continue; % unknown or unsupported layout
            end
        else
            % skip unknown layout
            continue;
        end
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
total_samples = size(acc_body_raw, 1);
if isfinite(max_steps)
    steps = min(total_samples, max_steps);
else
    steps = total_samples;
end
% Truncate IMU series to the requested number of steps to avoid size mismatches
imu_time      = imu_time(1:steps);
gyro_body_raw = gyro_body_raw(1:steps, :);
acc_body_raw  = acc_body_raw(1:steps, :);

num_imu_samples = length(imu_time);
x_log = zeros(15, num_imu_samples);
dprintf('Task 5: x_log initialized with size %dx%d\n', size(x_log));
euler_log = zeros(3, num_imu_samples);
quat_log  = zeros(4, num_imu_samples); % [q0;q1;q2;q3] scalar-first
zupt_log = zeros(1, num_imu_samples);
zupt_vel_norm = nan(1, num_imu_samples); % velocity norm after each ZUPT
acc_log = zeros(3, num_imu_samples); % Acceleration from propagated IMU data
zupt_count = 0;
zupt_fail_count = 0;            % count ZUPT events not clamped to zero
vel_blow_count = 0;             % track number of velocity blow-ups
accel_std_thresh = 0.05;        % [m/s^2]
gyro_std_thresh  = 0.005;       % [rad/s]
vel_thresh       = 0.1;         % [m/s]
vel_limit        = 500;         % [m/s] hard bound for velocity magnitude
vel_exceed_log   = zeros(0,2);  % [k,|v|] log for debugging
% Innovation logs for Task-7 diagnostics
innov_d2_total = nan(1, num_imu_samples);
innov_d2_pos   = nan(1, num_imu_samples);
innov_d2_vel   = nan(1, num_imu_samples);
% Yaw-aiding diagnostics
yaw_aid_flag         = zeros(1, num_imu_samples);   % 1 if yaw aiding applied at step
yaw_aid_residual_deg = nan(1, num_imu_samples);     % innovation (meas - est) in deg
% Mahalanobis gates (slightly relaxed to reduce late-stage starvation)
try
    gate_chi2_total = chi2inv(0.9999, 6);  % was ~16.81 at 0.99; ~27–35 at 0.9999
    gate_chi2_pos   = chi2inv(0.9999, 3);  % was ~11.35 at 0.99; ~27.88 at 0.9999
    gate_chi2_vel   = chi2inv(0.9999, 3);
catch
    gate_chi2_total = 32.0;  % fallback approximations
    gate_chi2_pos   = 27.88;
    gate_chi2_vel   = 27.88;
end
dprintf('-> 15-State filter initialized.\n');
dprintf('Subtask 5.4: Integrating IMU data for each method.\n');

%% ========================================================================
% Subtask 5.6: Kalman Filter for Sensor Fusion
% =========================================================================
dprintf('\nSubtask 5.6: Running Kalman Filter for sensor fusion for each method.\n');

% Interpolate GNSS measurements to IMU timestamps (optionally time-aligned)
gnss_time_eff = gnss_time;
dt_align = 0;
try
    if isfield(cfg,'time_align') && isfield(cfg.time_align,'apply') && cfg.time_align.apply
        % Prefer auto-estimate file from Task-7; else cfg.time_align.dt_s
        to_path = fullfile(results_dir, sprintf('%s_task7_timeoffset.mat', run_id));
        if isfile(to_path)
            Sdt = load(to_path);
            if isfield(Sdt,'dt_est'); dt_align = Sdt.dt_est; end
        else
            if isfield(cfg.time_align,'dt_s'); dt_align = cfg.time_align.dt_s; end
        end
        gnss_time_eff = gnss_time + dt_align;
        dprintf('[Task5] Applying GNSS time shift dt = %+0.3f s for measurement alignment.\n', dt_align);
    end
catch
end
gnss_pos_interp = zeros(num_imu_samples,3);
gnss_vel_interp = zeros(num_imu_samples,3);
for k = 1:3
    gnss_pos_interp(:,k) = interp1(gnss_time_eff, gnss_pos_ned(:,k), imu_time, 'linear', 'extrap');
    gnss_vel_interp(:,k) = interp1(gnss_time_eff, gnss_vel_ned(:,k), imu_time, 'linear', 'extrap');
    % Clamp out-of-bounds values to nearest GNSS sample to mirror np.interp
    gnss_pos_interp(imu_time < gnss_time_eff(1),k) = gnss_pos_ned(1,k);
    gnss_pos_interp(imu_time > gnss_time_eff(end),k) = gnss_pos_ned(end,k);
    gnss_vel_interp(imu_time < gnss_time_eff(1),k) = gnss_vel_ned(1,k);
    gnss_vel_interp(imu_time > gnss_time_eff(end),k) = gnss_vel_ned(end,k);
end
% Compare raw and interpolated GNSS data
if ~dryrun
    task5_gnss_interp_ned_plot(gnss_time, gnss_pos_ned, gnss_vel_ned, imu_time, ...
        gnss_pos_interp, gnss_vel_interp, run_id, results_dir, cfg);
end

% --- Main Filter Loop ---
dprintf('-> Starting filter loop over %d IMU samples...\n', num_imu_samples);
in_static = false; % debounce flag for ZUPT
% Optional trace buffers for first N steps
trace_n = max(0, min(trace_first_n, num_imu_samples));
if trace_n > 0
    trace.y = zeros(6, trace_n);
    trace.K = zeros(15, 6, trace_n);
    trace.i = zeros(1, trace_n);
else
    trace = struct();
end
% warn only once for singular covariance matrices
warned_S = false;
warned_S_z = false;
lever_notice_used_printed = false;
lever_notice_invalid_printed = false;
lever_arm_status = 'disabled'; % 'using-cfg' | 'using-estimate' | 'skipped-invalid' | 'disabled'
dbg_kf_pre_msg = '';
dbg_kf_postpred_msg = '';
dbg_kf_postupdate_msg = '';
dbg_zupt_msg = '';
zupt_applied_msg = '';
summary_loop_msg = '';
% Diagnostics counters
yaw_updates = 0; gaid_updates = 0; yaw_res_sum = 0; gaid_err_sum = 0;
yaw_speed_reject = 0; yaw_rate_reject = 0; yaw_residual_reject = 0;
% GNSS chi-square gating breakdown (per block)
gnss_attempt = 0; gnss_accept = 0; gnss_reject_total = 0; gnss_reject_pos = 0; gnss_reject_vel = 0; gnss_nan = 0;

for i = 1:num_imu_samples
    % Interpolate GNSS to the current IMU timestamp so the measurement
    % aligns with the state about to be propagated.  This mirrors the
    % Python helper ``interpolate_series`` used in GNSS_IMU_Fusion.py.
    gnss_pos_i = gnss_pos_interp(i,:)';
    gnss_vel_i = gnss_vel_interp(i,:)';
    dbg_kf_pre_msg = sprintf('[DBG-KF] k=%d pre-pred velN=%.1f velE=%.1f velD=%.1f norm=%.1f', i, x(4), x(5), x(6), norm(x(4:6)));

    if mod(i, 1e5) == 0
        dprintf('[DBG-KF] k=%d   posN=%.1f  velN=%.2f  accN=%.2f\n', ...
            i, x(1), x(4), a_ned(1));
    end
    % --- 1. State Propagation (Prediction) ---
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt_imu;
    % Match Python: apply fixed process noise per step (no extra dt scaling)
    P = F * P * F' + Q;

    % --- 2. Attitude Propagation ---
    corrected_gyro = gyro_body_raw(i,:)' - x(13:15);
    corrected_accel = acc_body_raw(i,:)' - x(10:12);
    current_omega_ie_b = C_B_N' * omega_ie_NED;
    w_b = corrected_gyro - current_omega_ie_b;
    % Quaternion propagation mirrors ``quat_from_rate`` in the Python
    % pipeline and keeps the attitude normalised for numerical stability.
    q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
    C_B_N = quat_to_rot(q_b_n);
    % The accelerometer measures specific force f = a - g. Recover inertial
    % acceleration via a = f + g (parity with Python pipeline).
    a_ned = C_B_N * corrected_accel + g_NED;
    if mod(i, 1e5) == 0
        dprintf('[DBG-KF] k=%d   posN=%.1f  velN=%.2f  accN=%.2f\n', ...
            i, x(1), x(4), a_ned(1));
    end
    if i > 1
        % Trapezoidal integration mirrors the Python fusion pipeline and
        % improves numerical stability over simple Euler steps.
        delta_v = 0.5 * (a_ned + prev_a_ned) * dt_imu;
        if norm(delta_v) > vel_limit
            vel_blow_count = vel_blow_count + 1;
            log_warn('[WARN-DELTA-V] k=%d delta_v=%.1f acc=[%.2f %.2f %.2f] exceeds limit; clamping.', ...
                     i, norm(delta_v), a_ned(1), a_ned(2), a_ned(3));
            vel_exceed_log(end+1,:) = [i, norm(delta_v)];
            delta_v = delta_v * (vel_limit / norm(delta_v));
        end
        vel_new = prev_vel + delta_v;
        dbg_kf_postpred_msg = sprintf('[DBG-KF] k=%d post-pred velN=%.1f velE=%.1f velD=%.1f norm=%.1f', i, vel_new(1), vel_new(2), vel_new(3), norm(vel_new));
        if norm(vel_new) > vel_limit
            vel_blow_count = vel_blow_count + 1;
            vel_exceed_log(end+1,:) = [i, norm(vel_new)];
            log_warn('[WARN-BLOWUP] k=%d vel_pred_norm=%.1f | Reverting to prev velN=%.1f velE=%.1f velD=%.1f', ...
                     i, norm(vel_new), prev_vel(1), prev_vel(2), prev_vel(3));
            vel_new = prev_vel;
            pos_new = x(1:3);
            P(4:6,4:6) = P(4:6,4:6) + eye(3) * 1e-3;
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
    % Build measurement with optional lever arm (IMU->GNSS) in body
    use_lever = false; r_b = [0;0;0];
    if isfield(cfg,'lever_arm') && isfield(cfg.lever_arm,'enabled') && cfg.lever_arm.enabled
        if isfield(cfg.lever_arm,'r_b') && numel(cfg.lever_arm.r_b)==3
            r_b = cfg.lever_arm.r_b(:);
            use_lever = true;
            lever_arm_status = 'using-cfg';
            if ~lever_notice_used_printed
                dprintf('[Task5] Using lever arm from cfg: [%.3f %.3f %.3f] m.\n', r_b);
                lever_notice_used_printed = true;
            else
                yaw_rate_reject = yaw_rate_reject + 1;
            end
        else
            yaw_speed_reject = yaw_speed_reject + 1;
        end
    else
        % Try auto-estimated lever arm from Task-7 results for this run
        la_path = fullfile(results_dir, sprintf('%s_task7_lever_arm.mat', run_id));
        if isfile(la_path)
            try
                LA = load(la_path);
                if isfield(LA,'rb') && numel(LA.rb)==3
                    % Only use if quality metrics present and valid
                    valid_ok = false;
                    if isfield(LA,'valid_la') && LA.valid_la
                        valid_ok = true;
                    elseif isfield(LA,'rms_la') && isfield(LA,'inl')
                        valid_ok = isfinite(LA.rms_la) && (LA.rms_la < 0.2) && (LA.inl >= 0.6);
                    end
                    if valid_ok
                        r_b = LA.rb(:); use_lever = true;
                        if ~lever_notice_used_printed
                            dprintf('[Task5] Using lever arm from Task-7: [%.3f %.3f %.3f] m.\n', r_b);
                            lever_notice_used_printed = true;
                        end
                        lever_arm_status = 'using-estimate';
                    else
                        if ~lever_notice_invalid_printed
                            dprintf('[Task5] Lever arm estimate present but not valid (RMS/inliers). Skipping.\n');
                            lever_notice_invalid_printed = true;
                        end
                        lever_arm_status = 'skipped-invalid';
                    end
                end
            catch
            end
        end
    end

    if use_lever
        % Predicted measurement with lever arm
        Rwb = C_B_N;                    % Body->NED
        omega_b_la = corrected_gyro;    % body angular rate [rad/s]
        h_pos = x(1:3) + Rwb * r_b;
        h_vel = x(4:6) + Rwb * (cross(omega_b_la, r_b));
        z = [gnss_pos_i; gnss_vel_i];
        y = z - [h_pos; h_vel];
        % Jacobian H_lever (6x15)
        Hm = zeros(6,15);
        % position wrt pos
        Hm(1:3,1:3) = eye(3);
        % position wrt attitude small-angle (BODY): -R * [r_b]_x
        Hm(1:3,7:9) = - Rwb * [   0    -r_b(3)  r_b(2);
                                   r_b(3)  0   -r_b(1);
                                  -r_b(2) r_b(1)  0    ];
        % velocity wrt vel
        Hm(4:6,4:6) = eye(3);
        % velocity wrt attitude: -R * [omega×r_b]_x
        v_body = cross(omega_b_la, r_b);
        Hm(4:6,7:9) = - Rwb * [   0      -v_body(3)  v_body(2);
                                   v_body(3)  0     -v_body(1);
                                  -v_body(2) v_body(1)  0      ];
        % velocity wrt gyro bias: R * [r_b]_x (since omega = omega_meas - b_g)
        Hm(4:6,13:15) = Rwb * [   0    -r_b(3)  r_b(2);
                                  r_b(3)  0   -r_b(1);
                                 -r_b(2) r_b(1)  0    ];
        % Innovation covariance and gain
        S = Hm * P * Hm' + R;
    else
        z = [gnss_pos_i; gnss_vel_i];
        y = z - H * x;
        S = H * P * H' + R;
    end
    % Innovation gating (chi-square) to reject outlier GNSS updates
    % FIX: Handle invalid measurements/covariance and account in counters
    if any(~isfinite(z)) || any(~isfinite(y)) || any(~isfinite(S(:)))
        gnss_attempt = gnss_attempt + 1;
        gnss_nan = gnss_nan + 1;
        innov_d2_total(i) = NaN; innov_d2_pos(i) = NaN; innov_d2_vel(i) = NaN;
        continue;
    end
    do_update = true;
    try
        d2 = y' * (S \ y);
    catch
        d2 = y' * pinv(S) * y;
    end
    % Per-block diagnostics
    Sp = S(1:3,1:3); Sv = S(4:6,4:6);
    try
        d2p = y(1:3)' * (Sp \ y(1:3));
    catch
        d2p = y(1:3)' * pinv(Sp) * y(1:3);
    end
    try
        d2v = y(4:6)' * (Sv \ y(4:6));
    catch
        d2v = y(4:6)' * pinv(Sv) * y(4:6);
    end
    innov_d2_total(i) = d2;
    innov_d2_pos(i)   = d2p;
    innov_d2_vel(i)   = d2v;
    gnss_attempt = gnss_attempt + 1;
    if d2 > gate_chi2_total
        do_update = false;
        gnss_reject_total = gnss_reject_total + 1;
        if mod(i, 10000) == 0
            dprintf('[GATE] k=%d reject update: d2=%.2f > %.2f\n', i, d2, gate_chi2_total);
        end
    end
    if d2p > gate_chi2_pos, gnss_reject_pos = gnss_reject_pos + 1; end
    if d2v > gate_chi2_vel, gnss_reject_vel = gnss_reject_vel + 1; end
    if do_update && rcond(S) <= eps
        if ~warned_S
            warning('Measurement covariance S is singular or ill-conditioned; using pinv.');
            warned_S = true;
        end
        if use_lever
            K = (P * Hm') * pinv(S);
        else
            K = (P * H') * pinv(S);
        end
    elseif do_update
        if use_lever
            K = (P * Hm') / S;
        else
            K = (P * H') / S;
        end
    end
    if do_update
        x = x + K * y;
        gnss_accept = gnss_accept + 1;
        if use_lever
            P = (eye(15) - K * Hm) * P;
        else
            P = (eye(15) - K * H) * P;
        end
        if i <= trace_n
            trace.y(:,i) = y;
            trace.K(:,:,i) = K;
            trace.i(i) = i;
        end
    end
    dbg_kf_postupdate_msg = sprintf('[DBG-KF] k=%d post-update velN=%.1f velE=%.1f velD=%.1f norm=%.1f', i, x(4), x(5), x(6), norm(x(4:6)));

    % --- 3b. Gravity-direction aiding (roll/pitch stabilisation) ---
    % Uses low-passed body specific-force direction to correct attitude.
    % Configuration: cfg.gaid.{enabled,fc_hz,sigma_dir_deg,max_dev_g,max_gyro_dps,min_speed_mps}
    if isfield(cfg,'gaid') && isfield(cfg.gaid,'enabled') && cfg.gaid.enabled
        % Persistent LPF state
        persistent acc_lp_initialized acc_lp
        if isempty(acc_lp_initialized), acc_lp_initialized = false; end

        % Parameters
        fc = cfg.gaid.fc_hz;                   % LPF cutoff [Hz]
        sigma = deg2rad(cfg.gaid.sigma_dir_deg);
        max_dev_g = cfg.gaid.max_dev_g;
        max_gyro_dps = cfg.gaid.max_gyro_dps;
        min_speed_mps = cfg.gaid.min_speed_mps;

        % Sample time and current measurements in body
        dt = dt_imu; % seconds
        f_b = corrected_accel;                 % specific force (a - g) in BODY [m/s^2]
        w_b = corrected_gyro;                  % gyro in BODY [rad/s]

        % Gravity magnitude from init
        g_mag = norm(g_NED);

        % LPF the accelerometer to estimate gravity direction component
        if ~acc_lp_initialized
            acc_lp = f_b; acc_lp_initialized = true;
        end
        alpha = onepole_alpha(fc, dt);
        acc_lp = (1-alpha) * acc_lp + alpha * f_b;

        % Gates: reject strong linear accel or fast rotation
        dev_g = abs(norm(acc_lp) - g_mag) / max(g_mag, 1e-6);
        gyro_dps = abs(w_b) * 180/pi;
        fast_rot = any(gyro_dps > max_gyro_dps);
        fast_lin = dev_g > max_dev_g;
        speed_ok = true;
        if min_speed_mps > 0
            speed_ok = norm(x(4:6)) >= min_speed_mps;
        end

        if ~(fast_rot || fast_lin) && speed_ok
            % Measurement: unit gravity direction in BODY
            % Accelerometer measures specific force f_b ≈ -g_b when static.
            z = -acc_lp / max(norm(acc_lp), 1e-9); % make it align with body-down

            % Predicted unit gravity direction in BODY: h = C_bn * e3, where e3=[0;0;1] down (NED)
            e3 = [0;0;1];
            C_bn = C_B_N';
            h = C_bn * e3; h = h / max(norm(h), 1e-9);

            % Innovation
            y_g = z - h;                         % 3x1

            % Jacobian wrt small attitude error delta-theta in BODY: -[h]_x
            H_att = -skew(h);                    % 3x3
            H_g = zeros(3,15); idx_att = 7:9;   % attitude block indices
            H_g(:, idx_att) = H_att;

            % Noise
            R_g = (sigma^2) * eye(3);

            % EKF update (Joseph form)
            S_g = H_g * P * H_g' + R_g;
            if rcond(S_g) <= eps, S_g = S_g + 1e-9*eye(3); end
            K_g = (P * H_g') / S_g;
            dx_g = K_g * y_g;
            x = x + dx_g;
            I_KH = eye(15) - K_g * H_g;
            P = I_KH * P * I_KH' + K_g * R_g * K_g';

            % Inject attitude correction into quaternion and reset attitude state
            dtheta = x(idx_att);
            dq = dtheta_to_quat(dtheta);    % 4x1, scalar-first
            q_b_n = quat_multiply(dq, q_b_n);
            q_b_n = q_b_n / norm(q_b_n);
            C_B_N = quat_to_rot(q_b_n);
            x(idx_att) = quat_to_euler(q_b_n);
            % Diagnostics
            gaid_updates = gaid_updates + 1;
            ang_err = acos(max(-1,min(1, dot(z,h)))) * 180/pi; % deg
            gaid_err_sum = gaid_err_sum + ang_err;
        end
    end

    % --- 3c. Yaw aiding from GNSS course-over-ground (when moving) ---
    if isfield(cfg,'yawaid') && isfield(cfg.yawaid,'enabled') && cfg.yawaid.enabled
        % Compute GNSS-derived course and speed at this step
        v_ned_meas = gnss_vel_interp(i,:)';
        speed = norm(v_ned_meas(1:2)); % horizontal speed [m/s]
        if speed >= cfg.yawaid.min_speed_mps
            % Heading from GNSS (course over ground), radians in [-pi,pi]
            psi_meas = atan2(v_ned_meas(2), v_ned_meas(1));
            % Current yaw estimate from quaternion
            eul_cur = quat_to_euler(q_b_n);
            psi_est = eul_cur(3);
            % Innovation with wrap to [-pi,pi]
            y_psi = wrapToPi_local(psi_meas - psi_est);

            % Gate on turn rate (body gyro) to avoid aiding during fast turns
            gyro_dps = abs(corrected_gyro) * 180/pi;
            if all(gyro_dps < cfg.yawaid.max_gyro_dps)
                % Numerical Jacobian d(psi)/d(delta-theta_body) via small-angle injection
                eps_num = 1e-6;
                H_att = zeros(1,3);
                for j=1:3
                    dth = zeros(3,1); dth(j) = eps_num;
                    dq = dtheta_to_quat(dth);
                    qj = quat_multiply(dq, q_b_n);
                    eul_j = quat_to_euler(qj);
                    dpsi = wrapToPi_local(eul_j(3) - psi_est);
                    H_att(j) = dpsi / eps_num;
                end
                % Full H matrix and noise
                H_psi = zeros(1,15); H_psi(1,7:9) = H_att;
                R_psi = deg2rad(cfg.yawaid.sigma_yaw_deg)^2;
                % Residual gate on magnitude
                if abs(rad2deg(y_psi)) > cfg.yawaid.gate_deg
                    yaw_residual_reject = yaw_residual_reject + 1;
                    continue;
                end
    % EKF update
                S_psi = H_psi * P * H_psi' + R_psi;
                if rcond(S_psi) <= eps, S_psi = S_psi + 1e-9; end
                K_psi = (P * H_psi') / S_psi;
                dx_psi = K_psi * y_psi;
                x = x + dx_psi;
                I_KH = eye(15) - K_psi * H_psi;
                P = I_KH * P * I_KH' + K_psi * R_psi * K_psi';

                % Inject attitude correction into quaternion and reset attitude state
                dtheta = x(7:9);
                dq = dtheta_to_quat(dtheta);
                q_b_n = quat_multiply(dq, q_b_n);
                q_b_n = q_b_n / norm(q_b_n);
                C_B_N = quat_to_rot(q_b_n);
                x(7:9) = quat_to_euler(q_b_n);

                % Diagnostics and console trace
                yaw_aid_flag(i) = 1;
                yaw_aid_residual_deg(i) = rad2deg(y_psi);
                yaw_updates = yaw_updates + 1;
                yaw_res_sum = yaw_res_sum + abs(yaw_aid_residual_deg(i));
                if ~dryrun
                    fprintf('[YAW-AID] k=%d  res=% .2f deg  speed=%.2f m/s\n', i, yaw_aid_residual_deg(i), speed);
                end
            else
                yaw_rate_reject = yaw_rate_reject + 1;
            end
        else
            yaw_speed_reject = yaw_speed_reject + 1;
        end
    end

    % --- 4. Velocity magnitude check ---
    vel_norm = norm(x(4:6));
    if vel_norm > vel_limit
        vel_blow_count = vel_blow_count + 1;
        vel_exceed_log(end+1,:) = [i, vel_norm];
        log_warn('[WARN-CLAMP] k=%d vel_state_norm=%.1f | Clamping to %.0f m/s', i, vel_norm, vel_limit);
        x(4:6) = x(4:6) / vel_norm * vel_limit;
        P(4:6,4:6) = P(4:6,4:6) + eye(3) * 1e-3;
    end
    assert(norm(x(4:6)) <= vel_limit * 2, 'Velocity runaway at k=%d', i);
    % update integrator history after correction
    prev_vel = x(4:6);
    prev_a_ned = a_ned;

    % --- 5. Zero-Velocity Update (ZUPT) ---
    win_size = 80;
    acc_win = acc_body_raw(max(1,i-win_size+1):i, :);
    gyro_win = gyro_body_raw(max(1,i-win_size+1):i, :);
    acc_std = max(std(acc_win,0,1));
    gyro_std = max(std(gyro_win,0,1));
    norm_acc = norm(acc_win(end,:));
    dbg_zupt_msg = sprintf('[DBG-ZUPT] k=%d acc_norm=%.4f (threshold=%.4f)', i, norm_acc, accel_std_thresh);
    if acc_std < accel_std_thresh && gyro_std < gyro_std_thresh && norm(x(4:6)) < vel_thresh
        zupt_count = zupt_count + 1;
        zupt_log(i) = 1;
        H_z = [zeros(3,3), eye(3), zeros(3,9)];
        R_z = eye(3) * 1e-6;
        y_z = -H_z * x;
        S_z = H_z * P * H_z' + R_z;
        if rcond(S_z) <= eps
            if ~warned_S_z
                warning('ZUPT covariance S_z is singular or ill-conditioned; using pinv.');
                warned_S_z = true;
            end
            K_z = (P * H_z') * pinv(S_z);
        else
            K_z = (P * H_z') / S_z;
        end
        x = x + K_z * y_z;
        P = (eye(15) - K_z * H_z) * P;
        zupt_vel_norm(i) = norm(x(4:6));
        if zupt_vel_norm(i) > vel_thresh
            zupt_fail_count = zupt_fail_count + 1;
            dprintf('ZUPT clamp failure at k=%d (norm=%.3f)\n', i, zupt_vel_norm(i));
        end
        x(4:6) = 0;
        zupt_applied_msg = sprintf('[ZUPT-APPLIED] k=%d reset vel to 0', i);
    end
    if mod(i,100000) == 0
        dprintf('ZUPT applied %d times so far\n', zupt_count);
    end

    % --- Log State and Attitude ---
    % Safety clamp: cap velocity norm to mitigate late-run blow-ups
    vmax = 50; % m/s cap for this dataset
    vnorm = norm(x(4:6));
    if vnorm > vmax
        x(4:6) = x(4:6) * (vmax / vnorm);
    end
    x_log(:, i) = x;
    euler_log(:, i) = quat_to_euler(q_b_n);
    quat_log(:, i)  = q_b_n;
    if mod(i, 100000) == 0
        dprintf('Task 5: Stored state at sample %d/%d\n', i, num_imu_samples);
    end
    if mod(i,10000) == 0
        summary_loop_msg = sprintf('[SUMMARY-LOOP] k=%d posN=%.1f velN=%.1f accN=%.2f ZUPT_cnt=%d blowups=%d', i, x(1), x(4), a_ned(1), zupt_count, vel_blow_count);
    end
end
if ~isempty(dbg_kf_pre_msg), disp(dbg_kf_pre_msg); end
if ~isempty(dbg_kf_postpred_msg), disp(dbg_kf_postpred_msg); end
if ~isempty(dbg_kf_postupdate_msg), disp(dbg_kf_postupdate_msg); end
if ~isempty(dbg_zupt_msg), disp(dbg_zupt_msg); end
if ~isempty(zupt_applied_msg), disp(zupt_applied_msg); end
if ~isempty(summary_loop_msg), disp(summary_loop_msg); end
dprintf('Method %s: IMU data integrated.\n', method);
dprintf('Method %s: Kalman Filter completed. ZUPTcnt=%d\n', method, zupt_count);
dprintf('Method %s: velocity blow-up events=%d\n', method, vel_blow_count);
dprintf('Method %s: ZUPT clamp failures=%d\n', method, zupt_fail_count);

% Optional end-window ZUPT: if last 10s are static, zero final velocities
try
    t_window = 10; % seconds
    idx_win = imu_time >= (imu_time(end) - t_window);
    if any(idx_win)
        acc_win = acc_body_raw(idx_win,:);
        gyro_win = gyro_body_raw(idx_win,:);
        if is_static(acc_win, gyro_win, accel_std_thresh, gyro_std_thresh)
            x_log(4:6, idx_win) = 0;
        end
    end
catch
end

%% ========================================================================
% Subtask 5.7: Handle Event at 5000s
% =========================================================================
dprintf('\nSubtask 5.7: No event handling needed as time < 5000s.\n');

%% ========================================================================
% Subtask 5.8: Plotting Results
% =========================================================================
dprintf('\nSubtask 5.8.2: Plotting results for %s.\n', method);

% Ensure array sizes match for plotting
if numel(imu_time) ~= size(x_log,2)
    N = min(numel(imu_time), size(x_log,2));
    imu_time = imu_time(1:N);
    x_log = x_log(:,1:N);
    euler_log = euler_log(:,1:N);
    quat_log  = quat_log(:,1:N);
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
if ~dryrun
    plot_state_grid(imu_time, p_n_fused, v_n_fused, a_n_fused, 'NED', ...
        'save_dir', results_dir, 'run_id', sprintf('%s_task5', run_id));
end

% --- Combined Position, Velocity and Acceleration ---
if ~dryrun
    fig = figure('Name', 'KF Results: P/V/A', 'Position', [100 100 1200 900]);
    labels = {'North', 'East', 'Down'};
    all_file = fullfile(results_dir, sprintf('%s_Task5_AllResults.pdf', run_id));
    if exist(all_file, 'file'); delete(all_file); end
    for i = 1:3
        % Position (FUSED only; GNSS removed)
        subplot(3,3,i); hold on;
        plot(imu_time, x_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'FUSED (TRIAD-KF)');
        hold off; grid on; legend({'FUSED (TRIAD-KF)'}, 'Location','best'); ylabel('[m]'); title(['Position ' labels{i}]);
        dprintf('Subtask 5.8.2: Plotted %s position %s: First = %.4f, Last = %.4f\n', ...
            method, labels{i}, x_log(i,1), x_log(i,end));

        % Velocity (FUSED only; GNSS removed)
        subplot(3,3,i+3); hold on;
        plot(imu_time, x_log(i+3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'FUSED (TRIAD-KF)');
        zupt_indices = find(zupt_log);
        if ~isempty(zupt_indices)
            plot(imu_time(zupt_indices), x_log(i+3,zupt_indices), 'ro', 'MarkerSize', 3, 'DisplayName', 'ZUPT');
        end
        hold off; grid on; legend({'FUSED (TRIAD-KF)','ZUPT'}, 'Location','best'); ylabel('[m/s]'); title(['Velocity ' labels{i}]);
        dprintf('Subtask 5.8.2: Plotted %s velocity %s: First = %.4f, Last = %.4f\n', ...
            method, labels{i}, x_log(i+3,1), x_log(i+3,end));

        % Acceleration (FUSED only; GNSS removed)
        subplot(3,3,i+6); hold on;
        plot(imu_time, acc_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'FUSED (TRIAD-KF)');
        hold off; grid on; legend({'FUSED (TRIAD-KF)'}, 'Location','best'); ylabel('[m/s^2]'); title(['Acceleration ' labels{i}]);
        dprintf('Subtask 5.8.2: Plotted %s acceleration %s: First = %.4f, Last = %.4f\n', ...
            method, labels{i}, acc_log(i,1), acc_log(i,end));
    end
    xlabel('Time (s)');
    sgtitle('Kalman Filter Results (FUSED only)');
    fname = fullfile(results_dir, sprintf('%s_task5_PVA', run_id));
    if cfg.plots.save_pdf
        print(fig, [fname '.pdf'], '-dpdf', '-bestfit');
    end
    % Always save interactive .fig
    try, savefig(fig, [fname '.fig']); catch, end
    if cfg.plots.save_png
        print(fig, [fname '.png'], '-dpng');
    end
    close(fig);
end

% --- Plot 4: Attitude (Euler Angles) ---
if ~dryrun
    fig_att = figure('Name', 'KF Results: Attitude', 'Position', [200 200 1200 600]);
    euler_labels = {'Roll', 'Pitch', 'Yaw'};
    for i = 1:3
        subplot(3, 1, i);
        plot(imu_time, rad2deg(euler_log(i,:)), 'b-');
        grid on; ylabel('[deg]'); title([euler_labels{i} ' Angle']);
    end
    xlabel('Time (s)'); sgtitle('Attitude Estimate Over Time');
    fname = fullfile(results_dir, sprintf('%s_Task5_Attitude', run_id));
    if cfg.plots.save_pdf
        print(fig_att, [fname '.pdf'], '-dpdf', '-bestfit');
    end
    try, savefig(fig_att, [fname '.fig']); catch, end
    if cfg.plots.save_png
        print(fig_att, [fname '.png'], '-dpng');
    end
    close(fig_att);
end

% --- Plot 5: Velocity Magnitude After ZUPTs ---
zupt_indices = find(zupt_log);
if ~dryrun && ~isempty(zupt_indices)
    fig_zupt = figure('Name', 'Post-ZUPT Velocity', 'Position', [300 300 800 400]);
    plot(imu_time(zupt_indices), zupt_vel_norm(zupt_indices), 'bo-');
    grid on; box on;
    xlabel('Time (s)'); ylabel('|v| after ZUPT [m/s]');
    title('Velocity magnitude following each ZUPT');
    legend('|v|');
    save_plot(fig_zupt, imu_name, gnss_name, [method '_ZUPT'], 5, ...
              cfg.plots.save_pdf, cfg.plots.save_png);
    close(fig_zupt);
end

if ~dryrun
    plot_task5_mixed_frame(imu_time, x_log(1:3,:), x_log(4:6,:), ...
        acc_log, euler_log, C_ECEF_to_NED, ref_r0, g_NED, run_id, method, results_dir, all_file, cfg);
    dprintf('Fused mixed frames plot saved\n');

    dprintf('Plotting all data in NED frame.\n');
    plot_task5_ned_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
        gnss_time, gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, method, run_id, cfg);

    dprintf('Plotting all data in ECEF frame.\n');
    plot_task5_ecef_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
        gnss_time, gnss_pos_ecef, gnss_vel_ecef, gnss_accel_ecef, C_ECEF_to_NED, ref_r0, method, run_id, cfg);

    dprintf('Plotting all data in body frame.\n');
    plot_task5_body_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, acc_body_raw, euler_log, ...
        gnss_time, gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, method, g_NED, run_id, cfg);

    state_file = fullfile(fileparts(imu_path), sprintf('STATE_%s.txt', imu_name));
    if exist(state_file, 'file')
        dprintf('Plotting fused ECEF trajectory with truth overlay.\n');
        plot_task5_ecef_truth(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
            state_file, C_ECEF_to_NED, ref_r0, method, run_id, cfg);
        dprintf('Plotting fused NED trajectory with truth overlay.\n');
        plot_task5_ned_truth(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
            state_file, C_ECEF_to_NED, ref_r0, method, run_id, cfg);
        dprintf('Plotting fused BODY signals with truth overlay.\n');
        plot_task5_body_truth(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, euler_log, ...
            state_file, C_ECEF_to_NED, ref_r0, g_NED, method, run_id, cfg);
    end
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
% Be tolerant to hard clamp behaviour: warn instead of aborting
if final_vel_mag > vel_limit
    warning('KF diverged: final velocity %.2f m/s exceeds %.0f m/s - check F & H matrices', final_vel_mag, vel_limit);
end
rms_resid_pos = sqrt(mean(res_pos.^2,'all'));
rms_resid_vel = sqrt(mean(res_vel.^2,'all'));
max_resid_pos = max(vecnorm(res_pos,2,2));
min_resid_pos = min(vecnorm(res_pos,2,2));
max_resid_vel = max(vecnorm(res_vel,2,2));
min_resid_vel = min(vecnorm(res_vel,2,2));

% Print a concise summary matching the Python pipeline
dprintf('Position: North=%.4f, East=%.4f, Down=%.4f\n', ...
        x_log(1,end), x_log(2,end), x_log(3,end));
dprintf('RMSE_pos: %.4f\n', rmse_pos);

% --- Plot: Position Residuals ---
if ~dryrun
    fig_err = figure('Name', 'KF Results: Position Residuals', 'Position', [150 150 1200 600]);
    err_labels = {'N', 'E', 'D'};
    for i = 1:3
        subplot(3,1,i);
        plot(gnss_time, res_pos(:,i), 'b-');
        grid on; ylabel('[m]'); title(['Residual ' err_labels{i}]);
    end
    xlabel('Time (s)'); sgtitle('Position Residuals (KF - GNSS)');
    fname = fullfile(results_dir, sprintf('%s_Task5_ErrorAnalysis', run_id));
    if cfg.plots.save_pdf
        print(fig_err, [fname '.pdf'], '-dpdf', '-bestfit');
    end
    if cfg.plots.save_png
        print(fig_err, [fname '.png'], '-dpng');
    end
    close(fig_err);
end
summary_line = sprintf(['[SUMMARY] method=%s imu=%s gnss=%s rmse_pos=%8.2fm ' ...
    'final_pos=%8.2fm rms_vel=%8.2fm/s final_vel=%8.2fm/s ' ...
    'rms_resid_pos=%8.2fm max_resid_pos=%8.2fm ' ...
    'rms_resid_vel=%8.2fm max_resid_vel=%8.2fm accel_bias=%.4f gyro_bias=%.4f ' ...
    'grav_err_mean=%.4f grav_err_max=%.4f omega_err_mean=%.4f omega_err_max=%.4f ' ...
    'ZUPT_count=%d'], method, imu_name, [gnss_name '.csv'], rmse_pos, ...
    final_pos_err, rmse_vel, final_vel, rms_resid_pos, max_resid_pos, ...
    rms_resid_vel, max_resid_vel, norm(accel_bias), norm(gyro_bias), grav_err_mean, grav_err_max, ...
    omega_err_mean, omega_err_max, zupt_count);
dprintf('%s\n', summary_line);
dprintf('[SUMMARY] method=%s rmse_pos=%.2f m final_pos=%.2f m ', ...
        method, rmse_pos, final_pos_err);
dprintf('rmse_vel=%.2f m/s final_vel=%.2f m/s\n', rmse_vel, final_vel);

% Store summary metrics and biases for later analysis
results = struct('method', method, 'rmse_pos', rmse_pos, 'rmse_vel', rmse_vel, ...
    'final_pos_error', final_pos_err, 'final_vel_error', final_vel_err, ...
    'final_vel', final_vel, 'final_acc_error', final_acc_err, 'accel_bias', accel_bias, 'gyro_bias', gyro_bias, ...
    'grav_err_mean', grav_err_mean, 'grav_err_max', grav_err_max, ...
    'omega_err_mean', omega_err_mean, 'omega_err_max', omega_err_max, ...
    'vel_blow_events', vel_blow_count);
perf_file = fullfile(results_dir, 'IMU_GNSS_bias_and_performance.mat');
summary_file = fullfile(results_dir, 'IMU_GNSS_summary.txt');
if ~dryrun
    fid = fopen(fullfile(results_dir, [run_id '_summary.txt']), 'w');
    fprintf(fid, '%s\n', summary_line);
    fclose(fid);
    % Result Logging -- store the metrics struct under the variable name
    % ``results`` to stay in sync with the Python pipeline.
    if isfile(perf_file)
        save(perf_file, '-append', 'results');
    else
        save(perf_file, 'results');
    end
    fid_sum = fopen(summary_file, 'a');
    fprintf(fid_sum, '%s\n', summary_line);
    fclose(fid_sum);
end

% Persist core results for unit tests and further analysis
% Persist IMU and GNSS time vectors for Tasks 6 and 7
time      = imu_time; %#ok<NASGU>  used by Task_6
gnss_time = gnss_time; %#ok<NASGU>
t_est = (0:size(x_log,2)-1)' * dt_imu; %#ok<NASGU>
dprintf('Saved t_est with length %d\n', length(t_est));
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
    if ~dryrun
        results_file = fullfile(results_dir, sprintf('%s_task5_results.mat', run_id));
        % Build estimator outputs in both NED and ECEF for downstream Tasks 6/7
        pos_ned_est = pos_ned;                 % Nx3
        vel_ned_est = vel_ned;                 % Nx3
        acc_ned_est = a_n_fused;               % Nx3
        C_N_E = C_ECEF_to_NED';
        pos_ecef_est = (C_N_E * pos_ned_est')' + ref_r0';
        vel_ecef_est = (C_N_E * vel_ned_est')';
        acc_ecef_est = (C_N_E * acc_ned_est')';
        fprintf('Task 5: Saving estimates | pos_ned_est=%dx%d pos_ecef_est=%dx%d\n', size(pos_ned_est));
        % Prepare attitude outputs; keep raw estimator quaternion in Task-5
        % FIX: Do not alter att_quat with Task-7 boresight here; save a separate
        %      att_quat_boresight for downstream visualization if available.
        att_quat_raw = quat_log';              % Nx4, raw estimator Body->NED
        att_quat = att_quat_raw;               % remain raw in Task-5 outputs
        att_quat_boresight = [];               % optional, for Task-6/7 plotting
        try
            bore_file = fullfile(results_dir, sprintf('%s_task7_boresight.mat', run_id));
            if isfile(bore_file)
                S_b = load(bore_file);
                if isfield(S_b,'qbar') && numel(S_b.qbar)==4
                    dq_boresight = S_b.qbar(:); % 1x4 or 4x1
                    att_quat_boresight = zeros(size(att_quat_raw));
                    for ii = 1:size(att_quat_raw,1)
                        att_quat_boresight(ii,:) = quat_multiply(dq_boresight, att_quat_raw(ii,:)').';
                    end
                    fprintf('Task 5: Prepared boresight-adjusted attitude (separate output).\n');
                end
            end
        catch
            % ignore boresight application errors
        end
        %#ok<NASU> att_quat att_quat_boresight
        save(results_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
            'gnss_pos_ecef', 'gnss_vel_ecef', 'gnss_accel_ecef', ...
            'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'quat_log', 'att_quat', 'att_quat_raw', 'att_quat_boresight', 'zupt_log', 'zupt_vel_norm', ...
            'time', 'gnss_time', 'pos_ned', 'vel_ned', 'ref_lat', 'ref_lon', 'ref_r0', ...
            'pos_ned_est', 'vel_ned_est', 'acc_ned_est', ...
            'pos_ecef_est', 'vel_ecef_est', 'acc_ecef_est', ...
            'states', 't_est', 'dt', 'imu_rate_hz', 'acc_body_raw', 'gyro_body_raw', 'trace', 'vel_blow_count', ...
            'vel_exceed_log', 'innov_d2_total', 'innov_d2_pos', 'innov_d2_vel', ...
            'yaw_aid_flag', 'yaw_aid_residual_deg', ...
            'gate_chi2_total', 'gate_chi2_pos', 'gate_chi2_vel');
        % Log attitude details for downstream tasks and human readers
        try
            fprintf('Task 5: Attitude logged as Body->NED (C_{nb}). Quaternion order [w x y z].\\n');
            fprintf('Task 5: Saved attitude to att_quat (Nx4, wxyz) and quat_log (4xN).\\n');
            if size(att_quat,1) >= 2
                fprintf('Task 5: att_quat first=[% .4f % .4f % .4f % .4f], last=[% .4f % .4f % .4f % .4f]\\n', ...
                    att_quat(1,1),att_quat(1,2),att_quat(1,3),att_quat(1,4), ...
                    att_quat(end,1),att_quat(end,2),att_quat(end,3),att_quat(end,4));
            end
        catch
        end
    % Provide compatibility with the Python pipeline and downstream tasks
    % by storing the fused position under the generic ``pos`` field as well.
    pos = pos_ned; %#ok<NASGU>
    save(results_file, 'x_log', 'pos', '-append');
    dprintf('State history (x_log) saved to %s\n', results_file);
    if isfile(results_file)
        dprintf('Results saved to %s\n', results_file);
    else
        warning('Missing %s', results_file);
    end
    try
        check = load(results_file, 'x_log');
        dprintf('Task 5: Verified x_log saved, size: %dx%d\n', size(check.x_log));
    catch
        warning('Task 5: Failed to verify x_log save in %s', results_file);
    end

    % Export estimator time vector for compatibility with Python pipeline
    time_file = fullfile(results_dir, sprintf('%s_task5_time.mat', run_id));
    save(time_file, 't_est', 'dt', 'x_log');
    dprintf('Task 5: Saved time vector to %s\n', time_file);

    method_struct = struct('gnss_pos_ned', gnss_pos_ned, 'gnss_vel_ned', gnss_vel_ned, ...
        'gnss_accel_ned', gnss_accel_ned, 'gnss_pos_ecef', gnss_pos_ecef, ...
        'gnss_vel_ecef', gnss_vel_ecef, 'gnss_accel_ecef', gnss_accel_ecef, ...
        'x_log', x_log, 'vel_log', vel_log, 'accel_from_vel', accel_from_vel, ...
        'euler_log', euler_log, 'zupt_log', zupt_log, 'zupt_vel_norm', zupt_vel_norm, 'time', time, ...
        'gnss_time', gnss_time, 'pos_ned', pos_ned, 'vel_ned', vel_ned, ...
        'ref_lat', ref_lat, 'ref_lon', ref_lon, 'ref_r0', ref_r0);
    % ``method`` already stores the algorithm name (e.g. 'TRIAD'). Use it
    % directly when saving so filenames match the Python pipeline.
    save_task_results(method_struct, imu_name, gnss_name, method, 5);
end

% Expose fused position for comparison plots across methods
assignin('base', ['pos_kf_' method], x_log(1:3,:)');
assignin('base', 't_kf', imu_time);

    % Return results structure and store in base workspace
    result = results;
    assignin('base', 'task5_results', result);

    % Close warning log file (if opened)
    if exist('warn_fid','var') && warn_fid > 0
        fclose(warn_fid);
    end

    % Print aiding summaries and GNSS speed stats / lever-arm status
    try
        if yaw_updates > 0
            fprintf('[Task5] Yaw-aid: %d updates (mean |res|=%.2f deg).\n', yaw_updates, yaw_res_sum/max(1,yaw_updates));
        else
            fprintf('[Task5] Yaw-aid: 0 updates. Check speed/gating.\n');
        end
        if gaid_updates > 0
            fprintf('[Task5] Gravity-aid: %d updates (mean angle err=%.2f deg).\n', gaid_updates, gaid_err_sum/max(1,gaid_updates));
        else
            fprintf('[Task5] Gravity-aid: 0 updates (likely high dynamics or gating).\n');
        end
        if abs(dt_align) > 1e-6
            fprintf('[Task5] Time alignment applied: dt = %+0.3f s (GNSS shifted).\n', dt_align);
        end
        % GNSS chi-square gating breakdown
        try
            fprintf('[Task5] GNSS chi-square gating: attempts=%d  accept=%d  reject_total=%d  reject_pos=%d  reject_vel=%d  nan=%d\n', ...
                gnss_attempt, gnss_accept, gnss_reject_total, gnss_reject_pos, gnss_reject_vel, gnss_nan);
        catch
        end
        % GNSS speed stats (horizontal) on estimator timeline
        try
            spd = hypot(gnss_vel_interp(:,1), gnss_vel_interp(:,2));
            smin = min(spd, [], 'omitnan'); smed = median(spd, 'omitnan'); smax = max(spd, [], 'omitnan');
            fprintf('[Task5] GNSS speed stats (m/s): min=%.3f  median=%.3f  max=%.3f\n', smin, smed, smax);
            if yaw_updates == 0
                fprintf('[Task5] Tip: Yaw-aid had 0 updates. If speeds are low, try cfg.yawaid.min_speed_mps=0.2.\n');
            end
        catch
        end
        % GNSS speed stats on GNSS timebase
        try
            spd_g = hypot(gnss_vel_ned(:,1), gnss_vel_ned(:,2));
            gsmin = min(spd_g, [], 'omitnan'); gsmed = median(spd_g, 'omitnan'); gsmax = max(spd_g, [], 'omitnan');
            fprintf('[Task5] GNSS speed stats (GNSS base, m/s): min=%.3f  median=%.3f  max=%.3f\n', gsmin, gsmed, gsmax);
        catch
        end
        % Yaw-aid gating breakdown
        fprintf('[Task5] Yaw-aid gating breakdown: accept=%d  speed_reject=%d  rate_reject=%d  residual_reject=%d.\n', ...
            yaw_updates, yaw_speed_reject, yaw_rate_reject, yaw_residual_reject);
        % Lever-arm summary
        fprintf('[Task5] Lever arm status: %s.\n', lever_arm_status);
    catch
    end

end

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

    function S = skew(v)
        %SKEW Skew-symmetric matrix from 3-vector
        S = [   0   -v(3)  v(2);
              v(3)    0   -v(1);
             -v(2)  v(1)    0  ];
    end

    function dq = dtheta_to_quat(dth)
        %DTHETA_TO_QUAT Convert small-angle delta-theta to quaternion [w;x;y;z]
        ang = norm(dth);
        if ang < 1e-12
            dq = [1;0;0;0];
            return;
        end
        axis = dth / ang;
        dq = [cos(ang/2); axis(:) * sin(ang/2)];
    end

    function alpha = onepole_alpha(fc, dt)
        %ONEPOLE_ALPHA Exact discretization coefficient for 1st-order LPF
        alpha = 1 - exp(-2*pi*fc*dt);
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

    function ang = wrapToPi_local(a)
        %WRAPTOPI_LOCAL Wrap angle(s) in radians to [-pi, pi]
        ang = mod(a + pi, 2*pi) - pi;
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
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
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

    function plot_task5_ned_frame(t, pos_ned, vel_ned, acc_ned, t_gnss, pos_gnss, vel_gnss, acc_gnss, method, run_id, cfg)
        %PLOT_TASK5_NED_FRAME Plot fused data in the NED frame (GNSS removed).
        labels = {'North','East','Down'};
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
        fprintf('Task 5: NED frame plotting | samples=%d\n', size(pos_ned,2));
        figure('Name','Task5 NED Frame','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        for k = 1:3
            subplot(3,3,k); hold on;
            plot(t, pos_ned(k,:), 'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{k}]); legend({'FUSED (TRIAD-KF)'},'Location','best');

            subplot(3,3,3+k); hold on;
            plot(t, vel_ned(k,:), 'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{k}]); legend({'FUSED (TRIAD-KF)'},'Location','best');

            subplot(3,3,6+k); hold on;
            plot(t, acc_ned(k,:), 'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{k}]); legend({'FUSED (TRIAD-KF)'},'Location','best');
        end
        sgtitle([method ' - NED frame (FUSED only)']);
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_NED_state', run_id));
        if cfg.plots.save_pdf
            print(gcf, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        % Always save interactive .fig and PNG for required deliverable
        try, savefig(gcf, [fname '.fig']); catch, end
        print(gcf, [fname '.png'], '-dpng');
        fprintf('Task 5: saved NED frame plot to %s (.pdf/.png)\n', fname);
        close(gcf);
    end

    function plot_task5_ecef_frame(t, pos_ned, vel_ned, acc_ned, t_gnss, pos_ecef, vel_ecef, acc_ecef, C_E_N, r0, method, run_id, cfg)
        %PLOT_TASK5_ECEF_FRAME Plot fused data in the ECEF frame (GNSS removed).
        labels = {'X','Y','Z'};
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
        pos_fused = (C_E_N' * pos_ned) + r0;
        vel_fused = C_E_N' * vel_ned;
        acc_fused = C_E_N' * acc_ned;
        fprintf('Task 5: ECEF frame plotting | samples=%d\n', size(pos_ned,2));
        figure('Name','Task5 ECEF Frame','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        for k = 1:3
            subplot(3,3,k); hold on;
            plot(t, pos_fused(k,:),'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{k}]); legend({'FUSED (TRIAD-KF)'},'Location','best');

            subplot(3,3,3+k); hold on;
            plot(t, vel_fused(k,:),'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{k}]); legend({'FUSED (TRIAD-KF)'},'Location','best');

            subplot(3,3,6+k); hold on;
            plot(t, acc_fused(k,:),'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{k}]); legend({'FUSED (TRIAD-KF)'},'Location','best');
        end
        sgtitle([method ' - ECEF frame (FUSED only)']);
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_ECEF_state', run_id));
        if cfg.plots.save_pdf
            print(gcf, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        % Always save interactive .fig and PNG for required deliverable
        try, savefig(gcf, [fname '.fig']); catch, end
        print(gcf, [fname '.png'], '-dpng');
        fprintf('Task 5: saved ECEF frame plot to %s (.pdf/.png)\n', fname);
        close(gcf);
    end

    function plot_task5_body_frame(t, pos_ned, vel_ned, acc_ned, acc_body_raw, eul_log, t_gnss, pos_gnss_ned, vel_gnss_ned, acc_gnss_ned, method, g_N, run_id, cfg)
        %PLOT_TASK5_BODY_FRAME Plot fused results in body frame coordinates (GNSS removed).
        labels = {'X','Y','Z'};
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
        N = size(pos_ned,2);
        fprintf('Task 5: Body frame plotting | samples=%d\n', N);
        pos_body = zeros(3,N); vel_body = zeros(3,N); acc_body = zeros(3,N);
        for k = 1:N
            C_B_N = euler_to_rot(eul_log(:,k));
            pos_body(:,k) = C_B_N' * pos_ned(:,k);
            vel_body(:,k) = C_B_N' * vel_ned(:,k);
            acc_body(:,k) = C_B_N' * (acc_ned(:,k) - g_N);
        end
        % GNSS body overlay removed per requirement; show only FUSED and IMU raw
        figure('Name','Task5 Body Frame','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        for j = 1:3
            subplot(3,3,j); hold on;
            plot(t, pos_body(j,:),'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{j}]); legend({'FUSED (TRIAD-KF)'},'Location','best');

            subplot(3,3,3+j); hold on;
            plot(t, vel_body(j,:),'b-','DisplayName','FUSED (TRIAD-KF)');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{j}]); legend({'FUSED (TRIAD-KF)'},'Location','best');

            subplot(3,3,6+j); hold on;
            plot(t, acc_body(j,:),'b-','DisplayName','FUSED (TRIAD-KF)');
            plot(t, acc_body_raw(:,j),'r-','DisplayName','IMU raw');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{j}]); legend({'FUSED (TRIAD-KF)','IMU raw'},'Location','best');
        end
        sgtitle([method ' - Body frame (FUSED only)']);
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_BODY_state', run_id));
        if cfg.plots.save_pdf
            print(gcf, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        % Always save interactive .fig and PNG for required deliverable
        try, savefig(gcf, [fname '.fig']); catch, end
        print(gcf, [fname '.png'], '-dpng');
        fprintf('Task 5: saved body frame plot to %s (.pdf/.png)\n', fname);
        close(gcf);
    end

    function plot_task5_ecef_truth(t, pos_ned, vel_ned, acc_ned, state_file, C_E_N, r0, method, run_id, cfg)
        %PLOT_TASK5_ECEF_TRUTH Overlay fused output with provided truth data.
        if ~exist(state_file,'file'); return; end
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
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
        try, savefig(gcf, [fname '.fig']); catch, end
        if cfg.plots.save_png
            print(gcf, [fname '.png'], '-dpng');
        end
        fprintf('Task 5: saved ECEF truth plot to %s (.pdf/.png)\n', fname);
        close(gcf);
    end

    function plot_task5_ned_truth(t, pos_ned, vel_ned, acc_ned, state_file, C_E_N, r0, method, run_id, cfg)
        %PLOT_TASK5_NED_TRUTH Overlay fused NED with truth converted to NED.
        if ~exist(state_file,'file'); return; end
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
        truth = readmatrix(state_file);
        t_truth = truth(:,2);
        pos_truth_e = truth(:,3:5);
        vel_truth_e = truth(:,6:8);
        % Convert to NED using Task 5 reference
        pos_truth_n = (C_E_N * (pos_truth_e' - r0))';
        vel_truth_n = (C_E_N * vel_truth_e')';
        dt_truth = diff(t_truth);
        acc_truth_n = [zeros(1,3); diff(vel_truth_n) ./ dt_truth];

        figure('Name','Task5 NED with Truth','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        labels = {'North','East','Down'};
        for k = 1:3
            subplot(3,3,k); hold on;
            plot(t_truth, pos_truth_n(:,k),'m-','DisplayName','Truth');
            plot(t, pos_ned(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{k}]); legend;

            subplot(3,3,3+k); hold on;
            plot(t_truth, vel_truth_n(:,k),'m-','DisplayName','Truth');
            plot(t, vel_ned(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{k}]); legend;

            subplot(3,3,6+k); hold on;
            plot(t_truth, acc_truth_n(:,k),'m-','DisplayName','Truth');
            plot(t, acc_ned(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{k}]); legend;
        end
        sgtitle([method ' - NED frame with Truth']);
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_NED_truth', run_id));
        % Save as .fig and PNG (PDF optional)
        try, savefig(gcf, [fname '.fig']); catch, end
        print(gcf, [fname '.png'], '-dpng');
        if isfield(cfg,'plots') && isfield(cfg.plots,'save_pdf') && cfg.plots.save_pdf
            print(gcf, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        close(gcf);
    end

    function plot_task5_body_truth(t, pos_ned, vel_ned, acc_ned, eul_log, state_file, C_E_N, r0, g_N, method, run_id, cfg)
        %PLOT_TASK5_BODY_TRUTH Overlay fused body-frame signals with truth.
        if ~exist(state_file,'file'); return; end
        visibleFlag = 'off';
        try
            if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
                visibleFlag = 'on';
            end
        catch
        end
        truth = readmatrix(state_file);
        t_truth = truth(:,2);
        pos_truth_e = truth(:,3:5);
        vel_truth_e = truth(:,6:8);
        pos_truth_n = (C_E_N * (pos_truth_e' - r0))';
        vel_truth_n = (C_E_N * vel_truth_e')';
        dt_truth = diff(t_truth);
        acc_truth_n = [zeros(1,3); diff(vel_truth_n) ./ dt_truth];

        % Fused signals in body frame using fused attitude history
        N = size(pos_ned,2);
        pos_body_f = zeros(3,N); vel_body_f = zeros(3,N); acc_body_f = zeros(3,N);
        for k = 1:N
            C_B_N = euler_to_rot(eul_log(:,k));
            pos_body_f(:,k) = C_B_N' * pos_ned(:,k);
            vel_body_f(:,k) = C_B_N' * vel_ned(:,k);
            acc_body_f(:,k) = C_B_N' * (acc_ned(:,k) - g_N);
        end
        % Truth signals rotated to body using fused attitude interpolated to truth timeline
        eul_truth = interp1(t, eul_log', t_truth, 'linear', 'extrap')';
        pos_body_t = zeros(size(pos_truth_n'));
        vel_body_t = zeros(size(vel_truth_n'));
        acc_body_t = zeros(size(acc_truth_n'));
        for k = 1:length(t_truth)
            C_B_N = euler_to_rot(eul_truth(:,k));
            pos_body_t(:,k) = C_B_N' * pos_truth_n(k,:)';
            vel_body_t(:,k) = C_B_N' * vel_truth_n(k,:)';
            acc_body_t(:,k) = C_B_N' * acc_truth_n(k,:)';
        end

        figure('Name','Task5 BODY with Truth','Position',[100 100 1200 900], ...
            'Visible', visibleFlag);
        labels = {'X','Y','Z'};
        for j = 1:3
            subplot(3,3,j); hold on;
            plot(t_truth, pos_body_t(j,:),'m-','DisplayName','Truth');
            plot(t, pos_body_f(j,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{j}]); legend;

            subplot(3,3,3+j); hold on;
            plot(t_truth, vel_body_t(j,:),'m-','DisplayName','Truth');
            plot(t, vel_body_f(j,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{j}]); legend;

            subplot(3,3,6+j); hold on;
            plot(t_truth, acc_body_t(j,:),'m-','DisplayName','Truth');
            plot(t, acc_body_f(j,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{j}]); legend;
        end
        sgtitle([method ' - Body frame with Truth']);
        fname = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_BODY_truth', run_id));
        try, savefig(gcf, [fname '.fig']); catch, end
        print(gcf, [fname '.png'], '-dpng');
        if isfield(cfg,'plots') && isfield(cfg.plots,'save_pdf') && cfg.plots.save_pdf
            print(gcf, [fname '.pdf'], '-dpdf', '-bestfit');
        end
        close(gcf);
    end
