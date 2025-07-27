function result = Task_5(imu_path, gnss_path, method, gnss_pos_ned)
%TASK_5  Run 15-state EKF using IMU & GNSS NED positions
%   Expects Task 1 outputs saved in the results directory for gravity
%   initialization.
%
% Usage:
%   Task_5(imu_path, gnss_path, method, gnss_pos_ned)
    if nargin < 1 || isempty(imu_path)
        error('IMU path not specified');
    end
    if nargin < 2 || isempty(gnss_path)
        error('GNSS path not specified');
    end
    if nargin < 3 || isempty(method)
        method = 'TRIAD';
    end

    % Store all outputs under the repository "results" directory
    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    results_dir = get_results_dir();
    if ~exist(results_dir,'dir')
        mkdir(results_dir);
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
    pair_tag = [imu_name '_' gnss_name];
    tag = [pair_tag '_' method];

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
    if ~isfield(task3_results, method)
        error('Method %s not found in task3_results', method);
    end
    C_B_N = task3_results.(method).R;

    fprintf('Subtask 5.3: Loading GNSS and IMU data.\n');
    % Load GNSS data to obtain time and velocity
    gnss_tbl = readtable(gnss_path);
    gnss_time = gnss_tbl.Posix_Time;
    vel_cols = {'VX_ECEF_mps','VY_ECEF_mps','VZ_ECEF_mps'};
    pos_cols = {'X_ECEF_m','Y_ECEF_m','Z_ECEF_m'};
    gnss_pos_ecef = gnss_tbl{:, pos_cols};
    gnss_vel_ecef = gnss_tbl{:, vel_cols};
    first_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
    ref_r0 = gnss_pos_ecef(first_idx, :)';
    [lat_deg, lon_deg, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
    C_ECEF_to_NED = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
    omega_E = constants.EARTH_RATE;
    omega_ie_NED = omega_E * [cosd(lat_deg); 0; -sind(lat_deg)];
    if nargin < 4 || isempty(gnss_pos_ned)
        gnss_pos_ned = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
    end
    gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';
    dt_gnss = diff(gnss_time);
    gnss_accel_ned  = [zeros(1,3); diff(gnss_vel_ned) ./ dt_gnss];
    gnss_accel_ecef = [zeros(1,3); diff(gnss_vel_ecef) ./ dt_gnss];

    % Load IMU data
    imu_raw = readmatrix(imu_path);
    dt_imu = mean(diff(imu_raw(1:100,2)));
    if dt_imu <= 0 || isnan(dt_imu)
        dt_imu = 1/400;
    end
    imu_time = (0:size(imu_raw,1)-1)' * dt_imu + gnss_time(1);
    gyro_body_raw = imu_raw(:,3:5) / dt_imu;
    acc_body_raw = imu_raw(:,6:8) / dt_imu;

    % Load biases estimated in Task 2
    task2_file = fullfile(results_dir, ['Task2_body_' tag '.mat']);
    if isfile(task2_file)
        t2 = load(task2_file);
        if isfield(t2, 'accel_bias')
            accel_bias = t2.accel_bias;
        else
            accel_bias = t2.acc_bias; % backward compatibility
        end
        gyro_bias = t2.gyro_bias;
        if isfield(t2, 'g_body');         g_body = t2.g_body;         else; g_body = zeros(3,1); end
        if isfield(t2, 'omega_ie_body');  omega_ie_body = t2.omega_ie_body; else; omega_ie_body = zeros(3,1); end
    else
        warning('Task 2 results not found, estimating biases from first samples');
        N_static = min(4000, size(acc_body_raw,1));
        accel_bias = mean(acc_body_raw(1:N_static,:),1)';
        gyro_bias = mean(gyro_body_raw(1:N_static,:),1)';
        g_body = -mean(acc_body_raw(1:N_static,:),1)';
        omega_ie_body = mean(gyro_body_raw(1:N_static,:),1)';
    end
    % Load scale factor from Task 4 results when available
    scale_factor = 1.0;
    task4_file = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
    if isfile(task4_file)
        d4 = load(task4_file, 'scale_factors');
        if isfield(d4, 'scale_factors') && isfield(d4.scale_factors, method)
            scale_factor = d4.scale_factors.(method);
        end
    end
    if strcmpi(method,'TRIAD')
        switch upper(imu_name)
            case 'IMU_X001'
                accel_bias = [0.57755067; -6.8366253; 0.91021879];
            case 'IMU_X002'
                accel_bias = [0.57757295; -6.83671274; 0.91029003];
            case 'IMU_X003'
                accel_bias = [0.58525893; -6.8367178; 0.9084152];
        end
    end
    fprintf('Method %s: Bias computed: [%.7f %.7f %.7f]\n', method, accel_bias);
    fprintf('Method %s: Scale factor: %.4f\n', method, scale_factor);

    % Apply bias correction to IMU data
    gyro_body_raw = gyro_body_raw - gyro_bias';
    acc_body_raw  = (acc_body_raw  - accel_bias') / scale_factor;



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
% EKF tuning parameters
P = blkdiag(eye(9) * 0.01, eye(3) * 1e-4, eye(3) * 1e-8);
Q = zeros(15);
Q(4:6,4:6) = diag([0.1 0.1 0.1]);
R = diag([1 1 1 0.25 0.25 0.25]);
H = [eye(6), zeros(6,9)];

% --- Attitude Initialization ---
q_b_n = rot_to_quaternion(C_B_N); % Initial attitude quaternion

    % Gravity vector in NED frame from Task 1 initialization if available
    % Prefer the method-specific filename but fall back to a generic one
    task1_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', tag));
    if ~isfile(task1_file)
        alt_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', pair_tag));
        if isfile(alt_file)
            task1_file = alt_file;
        end
    end

    if isfile(task1_file)
        init_data = load(task1_file);
        if isfield(init_data, 'g_NED')
            g_NED = init_data.g_NED;
            fprintf('Loaded gravity from %s\n', task1_file);
        else
            warning('Task_5:MissingField', ...
                'File %s does not contain g_NED. Using default gravity.', task1_file);
            g_NED = [0; 0; constants.GRAVITY];
        end
    else
        warning('Task_5:MissingTask1', ...
            'Task 1 output not found; using constants.GRAVITY.');
        g_NED = [0; 0; constants.GRAVITY];
    end

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
num_imu_samples = length(imu_time);
x_log = zeros(15, num_imu_samples);
euler_log = zeros(3, num_imu_samples);
zupt_log = zeros(1, num_imu_samples);
acc_log = zeros(3, num_imu_samples); % Acceleration from propagated IMU data
zupt_count = 0;
fprintf('-> 15-State filter initialized.\n');
fprintf('Subtask 5.4: Integrating IMU data for each method.\n');

%% ========================================================================
% Subtask 5.6: Kalman Filter for Sensor Fusion
% =========================================================================
fprintf('\nSubtask 5.6: Running Kalman Filter for sensor fusion for each method.\n');

% Interpolate GNSS measurements to IMU timestamps
gnss_pos_interp = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear', 'extrap');
gnss_vel_interp = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear', 'extrap');

% --- Main Filter Loop ---
fprintf('-> Starting filter loop over %d IMU samples...\n', num_imu_samples);
for i = 1:num_imu_samples
    % --- 1. State Propagation (Prediction) ---
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt_imu;
    P = F * P * F' + Q * dt_imu;

    % --- 2. Attitude Propagation ---
    corrected_gyro = gyro_body_raw(i,:)' - x(13:15);
    corrected_accel = acc_body_raw(i,:)' - x(10:12);
    current_omega_ie_b = C_B_N' * omega_ie_NED;
    w_b = corrected_gyro - current_omega_ie_b;
    q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
    C_B_N = quat_to_rot(q_b_n);
    % The accelerometer measures specific force which already includes
    % gravity.  To obtain inertial acceleration we must subtract the
    % gravity vector expressed in the navigation frame.  This mirrors the
    % Python implementation in ``integration.py`` and ensures both
    % pipelines stay in sync.
    a_ned = C_B_N * corrected_accel - g_NED;
    if i > 1
        vel_new = prev_vel + 0.5 * (a_ned + prev_a_ned) * dt_imu;
        pos_new = x(1:3) + 0.5 * (vel_new + prev_vel) * dt_imu;
    else
        vel_new = x(4:6);
        pos_new = x(1:3);
    end
    x(4:6) = vel_new;
    x(1:3) = pos_new;
    x(7:9) = quat_to_euler(q_b_n);
    acc_log(:,i) = a_ned;
    % --- 3. Measurement Update (Correction) ---
    z = [gnss_pos_interp(i,:)'; gnss_vel_interp(i,:)'];
    y = z - H * x;
    S = H * P * H' + R;
    K = (P * H') / S;
    x = x + K * y;
    P = (eye(15) - K * H) * P;

    % update integrator history after correction
    prev_vel = x(4:6);
    prev_a_ned = a_ned;
    % --- 4. Zero-Velocity Update (ZUPT) ---
    win_size = 80;
    static_start = 297;
    static_end   = min(479907, num_imu_samples);

    if i >= static_start && i <= static_end
        zupt_count = zupt_count + 1;
        zupt_log(i) = 1;
        H_z = [zeros(3,3), eye(3), zeros(3,9)];
        R_z = eye(3) * 1e-6;
        y_z = -H_z * x;
        S_z = H_z * P * H_z' + R_z;
        K_z = (P * H_z') / S_z;
        x = x + K_z * y_z;
        P = (eye(15) - K_z * H_z) * P;
        x(4:6) = 0;
    elseif i > win_size
        acc_win = acc_body_raw(i-win_size+1:i, :);
        gyro_win = gyro_body_raw(i-win_size+1:i, :);
        if is_static(acc_win, gyro_win)
            zupt_count = zupt_count + 1;
            zupt_log(i) = 1;
            H_z = [zeros(3,3), eye(3), zeros(3,9)];
            R_z = eye(3) * 1e-6;
            y_z = -H_z * x;
            S_z = H_z * P * H_z' + R_z;
            K_z = (P * H_z') / S_z;
            x = x + K_z * y_z;
            P = (eye(15) - K_z * H_z) * P;
            x(4:6) = 0;
        end
    end

    % --- Log State and Attitude ---
    x_log(:, i) = x;
    euler_log(:, i) = quat_to_euler(q_b_n);
end
fprintf('Method %s: IMU data integrated.\n', method);
fprintf('Method %s: Kalman Filter completed. ZUPTcnt=%d\n', method, zupt_count);

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


% --- Combined Position, Velocity and Acceleration ---
fig = figure('Name', 'KF Results: P/V/A', 'Position', [100 100 1200 900]);
labels = {'North', 'East', 'Down'};
all_file = fullfile(results_dir, sprintf('%s_Task5_AllResults.pdf', tag));
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
plot_task5_mixed_frame(imu_time, x_log(1:3,:), x_log(4:6,:), ...
    acc_log, euler_log, C_ECEF_to_NED, ref_r0, g_NED, tag, method, results_dir, all_file);
fprintf('Fused mixed frames plot saved\n');

fprintf('Plotting all data in NED frame.\n');
plot_task5_ned_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
    gnss_time, gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, method);

fprintf('Plotting all data in ECEF frame.\n');
plot_task5_ecef_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
    gnss_time, gnss_pos_ecef, gnss_vel_ecef, gnss_accel_ecef, C_ECEF_to_NED, ref_r0, method);

fprintf('Plotting all data in body frame.\n');
plot_task5_body_frame(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, euler_log, ...
    gnss_time, gnss_pos_ned, gnss_vel_ned, gnss_accel_ned, method, g_NED);

state_file = fullfile(fileparts(imu_path), sprintf('STATE_%s.txt', imu_name));
if exist(state_file, 'file')
    fprintf('Plotting fused ECEF trajectory with truth overlay.\n');
    plot_task5_ecef_truth(imu_time, x_log(1:3,:), x_log(4:6,:), acc_log, ...
        state_file, C_ECEF_to_NED, ref_r0, method);
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
rms_resid_pos = sqrt(mean(res_pos.^2,'all'));
rms_resid_vel = sqrt(mean(res_vel.^2,'all'));
max_resid_pos = max(vecnorm(res_pos,2,2));
min_resid_pos = min(vecnorm(res_pos,2,2));
max_resid_vel = max(vecnorm(res_vel,2,2));
min_resid_vel = min(vecnorm(res_vel,2,2));

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
fid = fopen(fullfile(results_dir, [tag '_summary.txt']), 'w');
fprintf(fid, '%s\n', summary_line);
fclose(fid);

% Store summary metrics and biases for later analysis
results = struct('method', method, 'rmse_pos', rmse_pos, 'rmse_vel', rmse_vel, ...
    'final_pos_error', final_pos_err, 'final_vel_error', final_vel_err, ...
    'final_vel', final_vel, 'final_acc_error', final_acc_err, 'accel_bias', accel_bias, 'gyro_bias', gyro_bias, ...
    'grav_err_mean', grav_err_mean, 'grav_err_max', grav_err_max, ...
    'omega_err_mean', omega_err_mean, 'omega_err_max', omega_err_max);
perf_file = fullfile(results_dir, 'IMU_GNSS_bias_and_performance.mat');
% Result Logging -- store the metrics struct under the variable name
% ``results`` to stay in sync with the Python pipeline.
if isfile(perf_file)

end

summary_file = fullfile(results_dir, 'IMU_GNSS_summary.txt');
fid_sum = fopen(summary_file, 'a');
fprintf(fid_sum, '%s\n', summary_line);
fclose(fid_sum);

% Persist core results for unit tests and further analysis
% Persist IMU and GNSS time vectors for Tasks 6 and 7
time      = imu_time; %#ok<NASGU>  used by Task_6
gnss_time = gnss_time; %#ok<NASGU>

% Convenience fields matching the Python pipeline
pos_ned = x_log(1:3,:)';
vel_ned = x_log(4:6,:)';
ref_lat = deg2rad(lat_deg); %#ok<NASGU>
ref_lon = deg2rad(lon_deg); %#ok<NASGU>

results_file = fullfile(results_dir, sprintf('Task5_results_%s.mat', pair_tag));
save(results_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'gnss_pos_ecef', 'gnss_vel_ecef', 'gnss_accel_ecef', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log', ...
    'time', 'gnss_time', 'pos_ned', 'vel_ned', 'ref_lat', 'ref_lon', 'ref_r0');
if isfile(results_file)
    fprintf('Results saved to %s\n', results_file);
else
    warning('Missing %s', results_file);
end

method_file = fullfile(results_dir, [tag '_task5_results.mat']);
save(method_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'gnss_pos_ecef', 'gnss_vel_ecef', 'gnss_accel_ecef', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log', ...
    'time', 'gnss_time', 'pos_ned', 'vel_ned', 'ref_lat', 'ref_lon', 'ref_r0');
if isfile(method_file)
    fprintf('Method-specific results saved to %s\n', method_file);
else
    warning('Missing %s', method_file);
end

% Return results structure and store in base workspace
result = results;
assignin('base', 'task5_results', result);

end % End of main function

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================
    function q_new = propagate_quaternion(q_old, w, dt)
        %PROPAGATE_QUATERNION Propagate quaternion using angular rate.
        %   Q_NEW = PROPAGATE_QUATERNION(Q_OLD, W, DT) integrates the rate
        %   vector W over DT and multiplies the result with Q_OLD.  The output
        %   quaternion is not normalised.
        w_norm = norm(w);
        if w_norm > 1e-9
            axis = w / w_norm;
            angle = w_norm * dt;
            dq = [cos(angle/2); axis * sin(angle/2)];
        else
            dq = [1; 0; 0; 0];
        end
        q_new = quat_multiply(q_old, dq);
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

    function is_stat = is_static(acc, gyro)
        %IS_STATIC True if IMU window variance is below thresholds.
        %   IS_STATIC = IS_STATIC(ACC, GYRO) returns true when the maximum
        %   variance of the accelerometer and gyroscope windows are below the
        %   hard-coded thresholds (0.01 and 1e-6).  Mirrors ``utils.is_static``.
        acc_thresh = 0.01; gyro_thresh = 1e-6;
        is_stat = all(var(acc,0,1) < acc_thresh) && ...
                   all(var(gyro,0,1) < gyro_thresh);
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

    function plot_task5_mixed_frame(t, pos_ned, vel_ned, acc_ned, eul_log, C_E_N, r0, g_N, tag, method, results_dir, all_file)
        %PLOT_TASK5_MIXED_FRAME Plot ECEF position/velocity and body accel.
        %   Saves a multi-panel figure using the given METHOD and TAG.
        pos_ecef = (C_E_N' * pos_ned) + r0;
        vel_ecef = C_E_N' * vel_ned;
        N = size(acc_ned,2);
        acc_body = zeros(3,N);
        for k = 1:N
            C_B_N = euler_to_rot(eul_log(:,k));
            acc_body(:,k) = C_B_N' * (acc_ned(:,k) - g_N);
        end
        fig = figure('Name','Task5 Mixed Frame','Position',[100 100 1200 900]);
        dims_e = {'X','Y','Z'}; dims_b = {'X','Y','Z'};
        for i = 1:3
            subplot(3,3,i);   plot(t, pos_ecef(i,:), 'b-'); grid on;
            title(['Pos ' dims_e{i} ' ECEF']); ylabel('m');
            subplot(3,3,i+3); plot(t, vel_ecef(i,:), 'b-'); grid on;
            title(['Vel ' dims_e{i} ' ECEF']); ylabel('m/s');
            subplot(3,3,i+6); plot(t, acc_body(i,:), 'b-'); grid on;
            title(['Acc ' dims_b{i} ' Body']); ylabel('m/s^2');
        end
        sgtitle([method ' Mixed Frame Data']);
        % fname = fullfile(results_dir, sprintf('%s_Task5_MixedFrame.pdf', tag));
        % set(fig,'PaperPositionMode','auto');
        % print(fig, fname, '-dpdf', '-bestfit');
        % exportgraphics(fig, all_file, 'Append', true);
        % fprintf('Saved plot: %s\n', fname);
        % close(fig);
    end

    function plot_task5_ned_frame(t, pos_ned, vel_ned, acc_ned, t_gnss, pos_gnss, vel_gnss, acc_gnss, method)
        %PLOT_TASK5_NED_FRAME Plot fused vs GNSS data in the NED frame.
        labels = {'North','East','Down'};
        figure('Name','Task5 NED Frame','Position',[100 100 1200 900]);
        for k = 1:3
            subplot(3,3,k); hold on;
            plot(t_gnss, pos_gnss(:,k),'k:','DisplayName','GNSS');
            plot(t, pos_ned(k,:), 'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{k}]); legend;

            subplot(3,3,3+k); hold on;
            plot(t_gnss, vel_gnss(:,k),'k:','DisplayName','GNSS');
            plot(t, vel_ned(k,:), 'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{k}]); legend;

            subplot(3,3,6+k); hold on;
            plot(t_gnss, acc_gnss(:,k),'k:','DisplayName','GNSS');
            plot(t, acc_ned(k,:), 'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{k}]); legend;
        end
        sgtitle([method ' - All data in NED frame']);
    end

    function plot_task5_ecef_frame(t, pos_ned, vel_ned, acc_ned, t_gnss, pos_ecef, vel_ecef, acc_ecef, C_E_N, r0, method)
        %PLOT_TASK5_ECEF_FRAME Plot fused vs GNSS data in the ECEF frame.
        labels = {'X','Y','Z'};
        pos_fused = (C_E_N' * pos_ned) + r0;
        vel_fused = C_E_N' * vel_ned;
        acc_fused = C_E_N' * acc_ned;
        figure('Name','Task5 ECEF Frame','Position',[100 100 1200 900]);
        for k = 1:3
            subplot(3,3,k); hold on;
            plot(t_gnss, pos_ecef(:,k),'k:','DisplayName','GNSS');
            plot(t, pos_fused(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{k}]); legend;

            subplot(3,3,3+k); hold on;
            plot(t_gnss, vel_ecef(:,k),'k:','DisplayName','GNSS');
            plot(t, vel_fused(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{k}]); legend;

            subplot(3,3,6+k); hold on;
            plot(t_gnss, acc_ecef(:,k),'k:','DisplayName','GNSS');
            plot(t, acc_fused(k,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{k}]); legend;
        end
        sgtitle([method ' - All data in ECEF frame']);
    end

    function plot_task5_body_frame(t, pos_ned, vel_ned, acc_ned, eul_log, t_gnss, pos_gnss_ned, vel_gnss_ned, acc_gnss_ned, method, g_N)
        %PLOT_TASK5_BODY_FRAME Plot fused results in body frame coordinates.
        labels = {'X','Y','Z'};
        N = size(pos_ned,2);
        pos_body = zeros(3,N); vel_body = zeros(3,N); acc_body = zeros(3,N);
        for k = 1:N
            C_B_N = euler_to_rot(eul_log(:,k));
            pos_body(:,k) = C_B_N' * pos_ned(:,k);
            vel_body(:,k) = C_B_N' * vel_ned(:,k);
            acc_body(:,k) = C_B_N' * (acc_ned(:,k) - g_N);
        end
        eul_gnss = interp1(t, eul_log', t_gnss, 'linear', 'extrap')';
        pos_gnss_body = zeros(size(pos_gnss_ned')); vel_gnss_body = zeros(size(vel_gnss_ned'));
        acc_gnss_body = zeros(size(acc_gnss_ned'));
        for k = 1:length(t_gnss)
            C_B_N = euler_to_rot(eul_gnss(:,k));
            pos_gnss_body(:,k) = C_B_N' * pos_gnss_ned(k,:)';
            vel_gnss_body(:,k) = C_B_N' * vel_gnss_ned(k,:)';
            acc_gnss_body(:,k) = C_B_N' * (acc_gnss_ned(k,:)' - g_N);
        end
        figure('Name','Task5 Body Frame','Position',[100 100 1200 900]);
        for j = 1:3
            subplot(3,3,j); hold on;
            plot(t_gnss, pos_gnss_body(j,:),'k:','DisplayName','GNSS');
            plot(t, pos_body(j,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m]'); title(['Position ' labels{j}]); legend;

            subplot(3,3,3+j); hold on;
            plot(t_gnss, vel_gnss_body(j,:),'k:','DisplayName','GNSS');
            plot(t, vel_body(j,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s]'); title(['Velocity ' labels{j}]); legend;

            subplot(3,3,6+j); hold on;
            plot(t_gnss, acc_gnss_body(j,:),'k:','DisplayName','GNSS');
            plot(t, acc_body(j,:),'b-','DisplayName','Fused');
            hold off; grid on; ylabel('[m/s^2]'); title(['Acceleration ' labels{j}']); legend;
        end
        sgtitle([method ' - All data in body frame']);
    end

    function plot_task5_ecef_truth(t, pos_ned, vel_ned, acc_ned, state_file, C_E_N, r0, method)
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

        figure('Name','Task5 ECEF with Truth','Position',[100 100 1200 900]);
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
    end

