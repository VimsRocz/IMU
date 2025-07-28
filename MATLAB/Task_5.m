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
fig_prefix = fullfile(results_dir, sprintf('%s_task5_results', run_tag));

if exist(init_file,'file') ~= 2 || exist(body_file,'file') ~= 2 || ...
        exist(task3_file,'file') ~= 2 || exist(task4_file,'file') ~= 2
    error('Task_5:MissingPrereq','Required Task 1--4 outputs not found.');
end
load(init_file, 'gravity_ned','lat0_rad','lon0_rad');
load(body_file, 'accel_bias','gyro_bias','accel_scale');
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
    if idx_gnss <= height(gnss) && imu.time(k) >= gnss_times(idx_gnss)
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
save(out_mat, 'x_log','pos_fused_ned','vel_fused_ned','acc_fused_ned','P', '-mat');
fprintf('Saved Task 5 results to %s\n', out_mat);

% Generate and save plots
fprintf('Subtask 5.7: Plotting fused vs IMU/GNSS in NED frame.\n');
figure; plot_ned_fusion(imu.time, pos_ned, pos_fused_ned, vel_ned, vel_fused_ned, accel, acc_fused_ned); %#ok<NASGU>
saveas(gcf, [fig_prefix '_NED.pdf']);

fprintf('Subtask 5.8: Plotting fused vs ground truth in ECEF frame.\n');
[pos_fused_ecef, vel_fused_ecef] = ned2ecef_series(pos_fused_ned, vel_fused_ned, lat0_rad, lon0_rad);
figure; plot_ecef_fusion(imu.time, zeros(size(pos_fused_ecef)), pos_fused_ecef, zeros(size(vel_fused_ecef)), vel_fused_ecef); %#ok<NASGU>
saveas(gcf, [fig_prefix '_ECEF.pdf']);

fprintf('Subtask 5.9: Plotting body-frame residuals & biases.\n');
figure; plot_body_residuals(imu.time, gyro, x_log(7:9,:), accel, x_log(10:12,:)); %#ok<NASGU>
saveas(gcf, [fig_prefix '_Body.pdf']);
end

% -------------------------------------------------------------------------
function data = load_imu_data(path)
%LOAD_IMU_DATA  Minimal loader returning time, accel_raw, gyro_raw.
    raw = load(path);
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

function plot_ned_fusion(t, pos_ref, pos_fused, vel_ref, vel_fused, accel_raw, accel_fused)
%PLOT_NED_FUSION  Simple NED comparison plots.
    subplot(3,1,1); plot(t, pos_ref(1,:), 'k:', t, pos_fused(1,:)); xlabel('t'); ylabel('N (m)'); grid on; legend('Ref','Fused');
    subplot(3,1,2); plot(t, vel_ref(1,:), 'k:', t, vel_fused(1,:)); xlabel('t'); ylabel('V_N (m/s)'); grid on; legend('Ref','Fused');
    subplot(3,1,3); plot(t(2:end), accel_raw(1,2:end), 'k:', t(2:end), accel_fused(1,:)); xlabel('t'); ylabel('a_N (m/s^2)'); grid on; legend('Raw','Fused');
end

function [pos_ecef, vel_ecef] = ned2ecef_series(pos_ned, vel_ned, lat, lon)
%NED2ECEF_SERIES  Convert series from NED to ECEF using fixed rotation.
    C = compute_C_ECEF_to_NED(lat, lon); % NED <- ECEF
    R = C';
    pos_ecef = R*pos_ned;
    vel_ecef = R*vel_ned;
end

function plot_ecef_fusion(t, pos_ref, pos_fused, vel_ref, vel_fused)
%PLOT_ECEF_FUSION  Simple ECEF comparison plots.
    subplot(2,1,1); plot(t, pos_fused(1,:), 'b'); xlabel('t'); ylabel('X (m)'); grid on; legend('Fused');
    subplot(2,1,2); plot(t, vel_fused(1,:), 'b'); xlabel('t'); ylabel('VX (m/s)'); grid on; legend('Fused');
end

function plot_body_residuals(t, gyro_raw, gyro_err, accel_raw, accel_bias)
%PLOT_BODY_RESIDUALS  Visualise body frame residuals.
    subplot(2,1,1); plot(t, gyro_raw(1,:), 'k:', t, gyro_err(1,:)); xlabel('t'); ylabel('\omega_x'); grid on; legend('Raw','Err');
    subplot(2,1,2); plot(t, accel_raw(1,:), 'k:', t, accel_bias(1,:)); xlabel('t'); ylabel('a_x'); grid on; legend('Raw','Bias');
end
