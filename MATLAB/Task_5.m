function Task_5(imu_path, gnss_path, method, dataset_tag)
%TASK_5  Sensor fusion: integrate IMU, correct with GNSS via 15-state Kalman filter
%   TASK_5(IMU_PATH, GNSS_PATH, METHOD) loads the intermediate results from
%   Tasks 1--4 and performs the full 15-state Kalman filter exactly as the
%   Python pipeline. Logging closely mirrors the Python implementation.
%
%   Diagnostic plots are generated in NED, ECEF and body frames and saved
%   under the ``MATLAB/results`` folder. The fused navigation state history
%   ``x_log`` is written to ``Task5_results_<tag>.mat`` for use by Tasks 6
%   and 7.
%
%   Usage:
%       Task_5('IMU_X002.dat', 'GNSS_X002.csv', 'TRIAD');
%
%   See also GET_RESULTS_DIR, load_imu_data, compute_state_transition,
%            compute_measurement_matrix, plot_ned_fusion, plot_ecef_fusion,
%            plot_body_residuals.

if nargin < 3
    method = '';
end
if nargin < 4 || isempty(dataset_tag)
    [~, imu_name, ~]  = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
    dataset_tag = sprintf('%s_%s', imu_name, gnss_name);
else
    [~, imu_name, ~]  = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
end

results_dir = get_results_dir();
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
run_tag = sprintf('%s_%s', dataset_tag, method);

fprintf('Subtask 5.1: Loading Task 1-4 outputs.\n');
% Build file paths
init_file = fullfile(results_dir, sprintf('Task1_%s_%s.mat', dataset_tag, method));
body_file = fullfile(results_dir, sprintf('Task2_%s_%s.mat', dataset_tag, method));
task3_file = fullfile(results_dir, sprintf('Task3_%s_%s.mat', dataset_tag, method));
task4_file = fullfile(results_dir, sprintf('Task4_%s_%s.mat', dataset_tag, method));
out_mat   = fullfile(results_dir, sprintf('Task5_%s_%s.mat', dataset_tag, method));
fig_ned   = fullfile(results_dir, sprintf('%s_task5_all_ned',  run_tag));
fig_ecef  = fullfile(results_dir, sprintf('%s_task5_all_ecef', run_tag));
fig_body  = fullfile(results_dir, sprintf('%s_task5_all_body', run_tag));

if exist(init_file,'file') ~= 2 || exist(body_file,'file') ~= 2 || ...
        exist(task3_file,'file') ~= 2 || exist(task4_file,'file') ~= 2
    error('Task_5:MissingPrereq','Required Task 1--4 outputs not found.');
end
load(init_file, 'gravity_ned','lat0_rad','lon0_rad');
load(body_file, 'accel_bias','gyro_bias');
tmp = load(body_file, 'accel_scale');
if isfield(tmp, 'accel_scale')
    accel_scale = tmp.accel_scale;
else
    accel_scale = 1;
    warning('Task_5:MissingField', 'Variable ''accel_scale'' not found. Using scale = 1.');
end
load(task3_file, 'task3_results');
C_b_n = task3_results.(method).R; %#ok<NASGU>
load(task4_file, 'pos_ned','vel_ned','acc_ned');

% Read raw sensor data
fprintf('Subtask 5.2: Reading IMU & GNSS data.\n');
imu  = load_imu_data(imu_path);
try
    gnss = read_csv_table(gnss_path);
catch e
    error('Failed to load GNSS file %s: %s', gnss_path, e.message);
end
dt   = median(diff(imu.time));
fprintf(' -> IMU dt = %.4f s, %d samples\n', dt, numel(imu.time));

% Correct IMU measurements
fprintf('Subtask 5.3: Applying accelerometer & gyro biases + scale.\n');
accel = (imu.accel_raw - accel_bias) * accel_scale;
gyro  = imu.gyro_raw - gyro_bias;
fprintf('   accel_bias = [%.4f %.4f %.4f], scale = %.4f\n', accel_bias, accel_scale);
fprintf('   gyro_bias  = [%.4e %.4e %.4e]\n', gyro_bias);

% Initialise Kalman filter
fprintf('Subtask 5.4: Configuring 15-state Kalman filter.\n');
n = numel(imu.time);
x = zeros(15,1);
P = eye(15)*1e0;
Q = blkdiag(zeros(6), eye(3)*1e-2, eye(3)*1e-6, eye(3)*1e-8);
R = blkdiag(eye(3)*0.25, eye(3)*0.25);
x_log = NaN(15,n);
fprintf('   Initial P(1:3,1:3)=%.1f, Q(4:6,4:6)=%.2f, R(1:3,1:3)=%.2f\n', P(1,1), Q(4,4), R(1,1));

gnss_times   = gnss.Posix_Time;
fprintf('Subtask 5.5: Running Kalman filter loop.\n');
idx_gnss = 1;
for k = 1:n
    % Predict
    F = compute_state_transition(gyro(:,k), dt);
    x = F*x;
    P = F*P*F' + Q;
    x_log(:,k) = x;
    % Update
    if idx_gnss <= length(gnss_times) && imu.time(k) >= gnss_times(idx_gnss)
        z_pos = [gnss.X_ECEF_m(idx_gnss); gnss.Y_ECEF_m(idx_gnss); gnss.Z_ECEF_m(idx_gnss)];
        z_vel = [gnss.VX_ECEF_mps(idx_gnss); gnss.VY_ECEF_mps(idx_gnss); gnss.VZ_ECEF_mps(idx_gnss)];
        H = compute_measurement_matrix(lat0_rad, lon0_rad);
        z = [z_pos; z_vel];
        y = z - H*x;
        S = H*P*H' + R;
        K = P*H'/S;
        x = x + K*y;
        P = (eye(15)-K*H)*P;
        idx_gnss = idx_gnss + 1;
        fprintf('   Update at sample %d: GNSS #%d\n', k, idx_gnss-1);
    end
end
fprintf('Task 5: Completed filter loop with %d GNSS updates.\n', idx_gnss-1);

% Extract fused estimates
fprintf('Subtask 5.6: Extracting fused pos/vel/acc.\n');
pos_fused_ned = x_log(1:3,:);
vel_fused_ned = x_log(4:6,:);
acc_fused_ned = diff(vel_fused_ned,1,2)/dt;
save(out_mat, 'x_log','pos_fused_ned','vel_fused_ned','acc_fused_ned','P');
fprintf('Saved Task 5 results to %s\n', out_mat);

% Generate and save plots
fprintf('Subtask 5.7: Plotting fused vs IMU/GNSS in NED frame.\n');
try
    % Check dimensions before plotting
    if size(pos_ned,2) == size(pos_fused_ned,2) && length(imu.time) == size(pos_fused_ned,2)
        fig = plot_task5_ned(imu.time, pos_ned, pos_fused_ned, vel_ned, vel_fused_ned, acc_ned, acc_fused_ned, method);
        print(fig, [fig_ned '.pdf'], '-dpdf', '-bestfit');
        print(fig, [fig_ned '.png'], '-dpng', '-r300');
        close(fig);
        fprintf('NED plot saved successfully.\n');
    else
        fprintf('Warning: Size mismatch in NED plot data - skipping plot.\n');
        fprintf('  time: %s, pos_ned: %s, pos_fused_ned: %s\n', ...
            mat2str(size(imu.time)), mat2str(size(pos_ned)), mat2str(size(pos_fused_ned)));
    end
catch e
    fprintf('Warning: NED plotting failed: %s\n', e.message);
end

fprintf('Subtask 5.8: Plotting fused vs ground truth in ECEF frame.\n');
try
    [pos_fused_ecef, vel_fused_ecef, acc_fused_ecef] = ned2ecef_series(pos_fused_ned, vel_fused_ned, acc_fused_ned, lat0_rad, lon0_rad);
    [pos_ref_ecef, vel_ref_ecef, acc_ref_ecef]   = ned2ecef_series(pos_ned, vel_ned, acc_ned, lat0_rad, lon0_rad);
    fig = plot_task5_ecef(imu.time, pos_ref_ecef, pos_fused_ecef, vel_ref_ecef, vel_fused_ecef, acc_ref_ecef, acc_fused_ecef, method);
    print(fig, [fig_ecef '.pdf'], '-dpdf', '-bestfit');
    print(fig, [fig_ecef '.png'], '-dpng', '-r300');
    close(fig);
    fprintf('ECEF plot saved successfully.\n');
catch e
    fprintf('Warning: ECEF plotting failed: %s\n', e.message);
end

fprintf('Subtask 5.9: Plotting body-frame residuals & biases.\n');
try
    fig = plot_task5_body(imu.time, gyro, x_log(7:9,:), accel, x_log(10:12,:), method);
    print(fig, [fig_body '.pdf'], '-dpdf', '-bestfit');
    print(fig, [fig_body '.png'], '-dpng', '-r300');
    close(fig);
    fprintf('Body plot saved successfully.\n');
catch e
    fprintf('Warning: Body plotting failed: %s\n', e.message);
end
end

% -------------------------------------------------------------------------
function data = load_imu_data(path)
%LOAD_IMU_DATA  Minimal loader returning time, accel_raw, gyro_raw.
    raw = read_matrix(path);
    data.time = raw(:,1);
    data.accel_raw = raw(:,2:4).';
    data.gyro_raw  = raw(:,5:7).';
end

function F = compute_state_transition(gyro, dt)
%COMPUTE_STATE_TRANSITION  Very small-angle approximation state matrix.
    F = eye(15);
    omega = norm(gyro);
    if omega > 0
        F(4:6,7:9) = -eye(3)*dt;
    end
end

function H = compute_measurement_matrix(lat, lon)
%COMPUTE_MEASUREMENT_MATRIX  Measurement matrix for GNSS position/velocity.
    H = [eye(3), zeros(3,12); zeros(3), eye(3), zeros(3,9)]; %#ok<EMACH>
end

function fig = plot_task5_ned(t, pos_ref, pos_fused, vel_ref, vel_fused, acc_ref, acc_fused, method)
%PLOT_TASK5_NED  3x3 comparison of GNSS vs fused results in NED frame.
    labels = {'N','E','D'};
    fig = figure('Name','Task5 NED');
    tl = tiledlayout(3,3,'TileSpacing','compact');
    for r = 1:3
        for c = 1:3
            ax = nexttile((r-1)*3+c); hold(ax,'on'); grid(ax,'on');
            switch r
                case 1
                    plot(ax, t, pos_ref(c,:), 'k-', 'DisplayName','Measured GNSS');
                    plot(ax, t, pos_fused(c,:), 'b-', 'DisplayName',['Fused (GNSS+IMU, ' method ')']);
                    ylabel(ax, sprintf(''Position %s (m)'', labels{c}));
                    title(ax, sprintf(''Pos %s'', labels{c}));
                case 2
                    plot(ax, t, vel_ref(c,:), 'k-', 'DisplayName','Measured GNSS');
                    plot(ax, t, vel_fused(c,:), 'b-', 'DisplayName',['Fused (GNSS+IMU, ' method ')']);
                    ylabel(ax, sprintf(''Velocity %s (m/s)'', labels{c}));
                    title(ax, sprintf(''Vel %s'', labels{c}));
                otherwise
                    plot(ax, t(2:end), acc_ref(c, :), 'k-', 'DisplayName','Measured GNSS');
                    plot(ax, t(2:end), acc_fused(c, :), 'b-', 'DisplayName',['Fused (GNSS+IMU, ' method ')']);
                    ylabel(ax, sprintf(''Accel %s (m/s^2)'', labels{c}));
                    title(ax, sprintf(''Acc %s'', labels{c}));
            end
            xlabel(ax,'Time (s)');
            legend(ax,'Location','best');
        end
    end
    title(tl, sprintf('Task 5  %s  NED Frame (Fused vs. Measured GNSS)', method));
end

function [pos_ecef, vel_ecef, acc_ecef] = ned2ecef_series(pos_ned, vel_ned, acc_ned, lat, lon)
%NED2ECEF_SERIES  Convert NED series to ECEF using fixed rotation.
    C = compute_C_ECEF_to_NED(lat, lon); % NED <- ECEF
    R = C';
    pos_ecef = R*pos_ned;
    vel_ecef = R*vel_ned;
    if nargin > 2 && ~isempty(acc_ned)
        acc_ecef = R*acc_ned;
    else
        acc_ecef = [];
    end
end

function fig = plot_task5_ecef(t, pos_ref, pos_fused, vel_ref, vel_fused, acc_ref, acc_fused, method)
%PLOT_TASK5_ECEF  3x3 comparison of GNSS vs fused results in ECEF frame.
    labels = {'X','Y','Z'};
    fig = figure('Name','Task5 ECEF');
    tl = tiledlayout(3,3,'TileSpacing','compact');
    for r = 1:3
        for c = 1:3
            ax = nexttile((r-1)*3+c); hold(ax,'on'); grid(ax,'on');
            switch r
                case 1
                    plot(ax, t, pos_ref(c,:), 'k-', 'DisplayName','Measured GNSS');
                    plot(ax, t, pos_fused(c,:), 'b-', 'DisplayName',['Fused (GNSS+IMU, ' method ')']);
                    ylabel(ax, sprintf(''Position %s (m)'', labels{c}));
                    title(ax, sprintf(''Pos %s'', labels{c}));
                case 2
                    plot(ax, t, vel_ref(c,:), 'k-', 'DisplayName','Measured GNSS');
                    plot(ax, t, vel_fused(c,:), 'b-', 'DisplayName',['Fused (GNSS+IMU, ' method ')']);
                    ylabel(ax, sprintf(''Velocity %s (m/s)'', labels{c}));
                    title(ax, sprintf(''Vel %s'', labels{c}));
                otherwise
                    plot(ax, t(2:end), acc_ref(c,:), 'k-', 'DisplayName','Measured GNSS');
                    plot(ax, t(2:end), acc_fused(c,:), 'b-', 'DisplayName',['Fused (GNSS+IMU, ' method ')']);
                    ylabel(ax, sprintf(''Accel %s (m/s^2)'', labels{c}));
                    title(ax, sprintf(''Acc %s'', labels{c}));
            end
            xlabel(ax,'Time (s)');
            legend(ax,'Location','best');
        end
    end
    title(tl, sprintf('Task 5  %s  ECEF Frame (Fused vs. Measured GNSS)', method));
end

function fig = plot_task5_body(t, gyro_raw, gyro_err, accel_raw, accel_bias, method)
%PLOT_TASK5_BODY  Plot body-frame residuals and biases.
    fig = figure('Name','Task5 Body');
    tl = tiledlayout(2,1,'TileSpacing','compact');
    ax = nexttile; hold(ax,'on'); grid(ax,'on');
    plot(ax, t, gyro_raw(1,:), 'k-', 'DisplayName','Gyro raw X');
    plot(ax, t, gyro_err(1,:), 'b-', 'DisplayName','Gyro bias X');
    ylabel(ax,'\omega_x (rad/s)'); legend(ax,'Location','best');
    ax = nexttile; hold(ax,'on'); grid(ax,'on');
    plot(ax, t, accel_raw(1,:), 'k-', 'DisplayName','Accel raw X');
    plot(ax, t, accel_bias(1,:), 'b-', 'DisplayName','Accel bias X');
    ylabel(ax,'a_x (m/s^2)'); legend(ax,'Location','best');
    xlabel(ax,'Time (s)');
    title(tl, sprintf('Task 5  %s  Body Frame Residuals', method));
end
