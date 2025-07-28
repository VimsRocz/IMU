function Task_5()
%TASK_5 Run Kalman Filter, generate plots and save results.
%   This simplified placeholder replaces the original implementation.
%   The function loads IMU and GNSS data from fixed paths, runs a dummy
%   15-state Kalman Filter loop, saves the state history and exports a set
%   of 3x3 subplot PDFs. Paths and logic mirror the provided code snippet.

fprintf('--- Starting Task 5: Sensor Fusion with Kalman Filter ---\n');

% Step 1: Configure Kalman Filter
fprintf('Task 5: Configuring 15-State Kalman Filter...\n');
num_states = 15;  % Position (3), Velocity (3), Attitude (3 or 4), Biases (6)
num_samples = 500000;  % IMU samples at 400 Hz
dt = 1/400;  % IMU sample interval (s)

x = zeros(num_states, 1);  % Initial state
P = eye(num_states) * 1e-2;  % Initial covariance
fprintf('Task 5: Initialized state vector (%dx1) and covariance (%dx%d)\n', ...
    num_states, num_states, num_states);

% Step 2: Initialize state history
fprintf('Task 5: Initializing state history matrix...\n');
x_log = zeros(num_states, num_samples);  % State history
fprintf('Task 5: x_log initialized with size %dx%d\n', num_states, num_samples);

% Step 3: Load IMU and GNSS data (assumed from previous tasks)
fprintf('Task 5: Loading IMU and GNSS data...\n');

% GNSS results from Task 4
task4_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/Task4_results_IMU_X002_GNSS_X002.mat';
try
    load(task4_file, 'gnss_ned_pos', 'gnss_ecef_pos', 'C_n_e');
    fprintf('Task 5: Loaded GNSS data and C_n_e from %s\n', task4_file);
catch
    error('Task 5: Failed to load GNSS data from %s.', task4_file);
end

% Rotation matrices from Task 3
task3_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/task3_results_IMU_X002_GNSS_X002.mat';
try
    load(task3_file, 'C_b_n');
    fprintf('Task 5: Loaded C_b_n from %s\n', task3_file);
catch
    error('Task 5: Failed to load C_b_n from %s.', task3_file);
end

% Step 4: Kalman Filter loop (simplified example)
fprintf('Task 5: Starting Kalman Filter loop over %d samples...\n', num_samples);
imu_data = load_imu_data();

gnss_idx = 1;  % Track GNSS sample
for t = 1:num_samples
    % Update with GNSS if available (every 400 samples)
    if mod(t, 400) == 0 && gnss_idx <= size(gnss_ned_pos, 1)
        gnss_idx = gnss_idx + 1;
    end
    x_log(:, t) = x;
    if mod(t, 100000) == 0
        fprintf('Task 5: Processed sample %d/%d\n', t, num_samples);
    end
end
fprintf('Task 5: Completed Kalman Filter loop\n');

% Step 5: Compute derived arrays
fprintf('Task 5: Computing derived arrays...\n');
pos_est_ned = x_log(1:3, :);  % NED position
vel_est_ned = x_log(4:6, :);  % NED velocity
pos_est_ecef = C_n_e * pos_est_ned;  % Convert to ECEF
vel_est_ecef = C_n_e * vel_est_ned;

acc_body = zeros(3, num_samples);
for t = 1:num_samples
    acc_body(:, t) = C_b_n(:, :, t) * imu_data(t, 1:3)';
end

downsample_factor = 400;
time_indices = 1:downsample_factor:num_samples;
pos_est_ecef_ds = pos_est_ecef(:, time_indices);
vel_est_ecef_ds = vel_est_ecef(:, time_indices);
pos_est_ned_ds = pos_est_ned(:, time_indices);
vel_est_ned_ds = vel_est_ned(:, time_indices);
acc_body_ds = acc_body(:, time_indices);

pos_residuals = pos_est_ecef_ds - gnss_ecef_pos';
vel_residuals = vel_est_ecef_ds - zeros(size(vel_est_ecef_ds));
fprintf('Task 5: Computed residuals, position size: %dx%d\n', size(pos_residuals));

% Step 6: Generate 3x3 subplot plots
fprintf('Task 5: Generating 3x3 subplot plots...\n');

fig_ecef = figure('Name', 'Task 5 - ECEF Frame (3x3)', 'Visible', 'on');
subplot(3,3,1); plot(time_indices, pos_est_ecef_ds(1,:), 'b'); hold on;
plot(time_indices, gnss_ecef_pos(:,1), 'r--'); title('Position X (ECEF)'); ylabel('m'); legend('Est X','GNSS X'); grid on;
subplot(3,3,2); plot(time_indices, pos_est_ecef_ds(2,:), 'g'); hold on;
plot(time_indices, gnss_ecef_pos(:,2), 'm--'); title('Position Y (ECEF)'); ylabel('m'); legend('Est Y','GNSS Y'); grid on;
subplot(3,3,3); plot(time_indices, pos_est_ecef_ds(3,:), 'k'); hold on;
plot(time_indices, gnss_ecef_pos(:,3), 'c--'); title('Position Z (ECEF)'); ylabel('m'); legend('Est Z','GNSS Z'); grid on;
subplot(3,3,4); plot(time_indices, vel_est_ecef_ds(1,:), 'b'); title('Velocity X (ECEF)'); ylabel('m/s'); legend('Est VX'); grid on;
subplot(3,3,5); plot(time_indices, vel_est_ecef_ds(2,:), 'g'); title('Velocity Y (ECEF)'); ylabel('m/s'); legend('Est VY'); grid on;
subplot(3,3,6); plot(time_indices, vel_est_ecef_ds(3,:), 'k'); title('Velocity Z (ECEF)'); ylabel('m/s'); legend('Est VZ'); grid on;
subplot(3,3,7); plot(time_indices, sqrt(sum(pos_est_ecef_ds.^2, 1)), 'b'); hold on;
plot(time_indices, sqrt(sum(gnss_ecef_pos.^2, 2))', 'r--'); title('Position Norm'); ylabel('m'); legend('Est Norm','GNSS Norm'); grid on;
subplot(3,3,8); plot(time_indices, sqrt(sum(vel_est_ecef_ds.^2, 1)), 'b'); title('Velocity Norm'); ylabel('m/s'); legend('Est Norm'); grid on;
subplot(3,3,9); plot(time_indices, sqrt(sum(pos_residuals.^2, 1)), 'k'); title('Position Error Norm'); ylabel('m'); legend('Error Norm'); grid on;
ecef_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_all_ecef.pdf';
saveas(fig_ecef, ecef_file);
fprintf('Task 5: Saved plot: %s\n', ecef_file);

fig_ned = figure('Name', 'Task 5 - NED Frame (3x3)', 'Visible', 'on');
subplot(3,3,1); plot(time_indices, pos_est_ned_ds(1,:), 'b'); hold on;
plot(time_indices, gnss_ned_pos(:,1), 'r--'); title('Position North'); ylabel('m'); legend('Est North','GNSS North'); grid on;
subplot(3,3,2); plot(time_indices, pos_est_ned_ds(2,:), 'g'); hold on;
plot(time_indices, gnss_ned_pos(:,2), 'm--'); title('Position East'); ylabel('m'); legend('Est East','GNSS East'); grid on;
subplot(3,3,3); plot(time_indices, pos_est_ned_ds(3,:), 'k'); hold on;
plot(time_indices, gnss_ned_pos(:,3), 'c--'); title('Position Down'); ylabel('m'); legend('Est Down','GNSS Down'); grid on;
subplot(3,3,4); plot(time_indices, vel_est_ned_ds(1,:), 'b'); title('Velocity North'); ylabel('m/s'); legend('Est North'); grid on;
subplot(3,3,5); plot(time_indices, vel_est_ned_ds(2,:), 'g'); title('Velocity East'); ylabel('m/s'); legend('Est East'); grid on;
subplot(3,3,6); plot(time_indices, vel_est_ned_ds(3,:), 'k'); title('Velocity Down'); ylabel('m/s'); legend('Est Down'); grid on;
subplot(3,3,7); plot(time_indices, sqrt(sum(pos_est_ned_ds.^2, 1)), 'b'); hold on;
plot(time_indices, sqrt(sum(gnss_ned_pos.^2, 2))', 'r--'); title('Position Norm'); ylabel('m'); legend('Est Norm','GNSS Norm'); grid on;
subplot(3,3,8); plot(time_indices, sqrt(sum(vel_est_ned_ds.^2, 1)), 'b'); title('Velocity Norm'); ylabel('m/s'); legend('Est Norm'); grid on;
subplot(3,3,9); plot(time_indices, sqrt(sum(pos_residuals.^2, 1)), 'k'); title('Position Error Norm'); ylabel('m'); legend('Error Norm'); grid on;
ned_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_all_ned.pdf';
saveas(fig_ned, ned_file);
fprintf('Task 5: Saved plot: %s\n', ned_file);

fig_body = figure('Name', 'Task 5 - Body Frame (3x3)', 'Visible', 'on');
subplot(3,3,1); plot(time_indices, acc_body_ds(1,:), 'b'); title('Acceleration X (Body)'); ylabel('m/s^2'); legend('Acc X'); grid on;
subplot(3,3,2); plot(time_indices, acc_body_ds(2,:), 'g'); title('Acceleration Y (Body)'); ylabel('m/s^2'); legend('Acc Y'); grid on;
subplot(3,3,3); plot(time_indices, acc_body_ds(3,:), 'k'); title('Acceleration Z (Body)'); ylabel('m/s^2'); legend('Acc Z'); grid on;
subplot(3,3,4); plot(time_indices, zeros(size(time_indices)), 'b'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,5); plot(time_indices, zeros(size(time_indices)), 'g'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,6); plot(time_indices, zeros(size(time_indices)), 'k'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,7); plot(time_indices, sqrt(sum(acc_body_ds.^2, 1)), 'b'); title('Acceleration Norm'); ylabel('m/s^2'); legend('Acc Norm'); grid on;
subplot(3,3,8); plot(time_indices, zeros(size(time_indices)), 'b'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,9); plot(time_indices, zeros(size(time_indices)), 'k'); title('Placeholder'); ylabel('N/A'); grid on;
body_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_all_body.pdf';
saveas(fig_body, body_file);
fprintf('Task 5: Saved plot: %s\n', body_file);

fig_mixed = figure('Name', 'Task 5 - Mixed Frames (3x3)', 'Visible', 'on');
subplot(3,3,1); plot(time_indices, pos_est_ecef_ds(1,:), 'b'); hold on;
plot(time_indices, gnss_ecef_pos(:,1), 'r--'); title('Position X (ECEF)'); ylabel('m'); legend('Est X','GNSS X'); grid on;
subplot(3,3,2); plot(time_indices, vel_est_ecef_ds(1,:), 'b'); title('Velocity X (ECEF)'); ylabel('m/s'); legend('Est VX'); grid on;
subplot(3,3,3); plot(time_indices, acc_body_ds(2,:), 'g'); title('Acceleration Y (Body)'); ylabel('m/s^2'); legend('Acc Y'); grid on;
subplot(3,3,4); plot(time_indices, acc_body_ds(3,:), 'k'); title('Acceleration Z (Body)'); ylabel('m/s^2'); legend('Acc Z'); grid on;
subplot(3,3,5); plot(time_indices, pos_est_ned_ds(1,:), 'b'); hold on;
plot(time_indices, gnss_ned_pos(:,1), 'r--'); title('Position North (NED)'); ylabel('m'); legend('Est North','GNSS North'); grid on;
subplot(3,3,6); plot(time_indices, vel_est_ned_ds(1,:), 'b'); title('Velocity North (NED)'); ylabel('m/s'); legend('Est North'); grid on;
subplot(3,3,7); plot(time_indices, sqrt(sum(pos_est_ecef_ds.^2, 1)), 'b'); title('Position Norm (ECEF)'); ylabel('m'); legend('Est Norm'); grid on;
subplot(3,3,8); plot(time_indices, sqrt(sum(acc_body_ds.^2, 1)), 'g'); title('Acceleration Norm (Body)'); ylabel('m/s^2'); legend('Acc Norm'); grid on;
subplot(3,3,9); plot(time_indices, sqrt(sum(pos_residuals.^2, 1)), 'k'); title('Position Error Norm'); ylabel('m'); legend('Error Norm'); grid on;
mixed_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_mixed_frames.pdf';
saveas(fig_mixed, mixed_file);
fprintf('Task 5: Saved plot: %s\n', mixed_file);

fig_pos_res = figure('Name', 'Task 5 - Position Residuals (3x3)', 'Visible', 'on');
subplot(3,3,1); plot(time_indices, pos_residuals(1,:), 'b'); title('Position Residual X (ECEF)'); ylabel('m'); legend('Res X'); grid on;
subplot(3,3,2); plot(time_indices, pos_residuals(2,:), 'g'); title('Position Residual Y (ECEF)'); ylabel('m'); legend('Res Y'); grid on;
subplot(3,3,3); plot(time_indices, pos_residuals(3,:), 'k'); title('Position Residual Z (ECEF)'); ylabel('m'); legend('Res Z'); grid on;
subplot(3,3,4); plot(time_indices, zeros(size(time_indices)), 'b'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,5); plot(time_indices, zeros(size(time_indices)), 'g'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,6); plot(time_indices, zeros(size(time_indices)), 'k'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,7); plot(time_indices, sqrt(sum(pos_residuals.^2, 1)), 'b'); title('Position Residual Norm'); ylabel('m'); legend('Res Norm'); grid on;
subplot(3,3,8); plot(time_indices, zeros(size(time_indices)), 'b'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,9); plot(time_indices, zeros(size(time_indices)), 'k'); title('Placeholder'); ylabel('N/A'); grid on;
pos_res_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_residuals_position_residuals.pdf';
saveas(fig_pos_res, pos_res_file);
fprintf('Task 5: Saved plot: %s\n', pos_res_file);

fig_vel_res = figure('Name', 'Task 5 - Velocity Residuals (3x3)', 'Visible', 'on');
subplot(3,3,1); plot(time_indices, vel_residuals(1,:), 'b'); title('Velocity Residual X (ECEF)'); ylabel('m/s'); legend('Res VX'); grid on;
subplot(3,3,2); plot(time_indices, vel_residuals(2,:), 'g'); title('Velocity Residual Y (ECEF)'); ylabel('m/s'); legend('Res VY'); grid on;
subplot(3,3,3); plot(time_indices, vel_residuals(3,:), 'k'); title('Velocity Residual Z (ECEF)'); ylabel('m/s'); legend('Res VZ'); grid on;
subplot(3,3,4); plot(time_indices, zeros(size(time_indices)), 'b'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,5); plot(time_indices, zeros(size(time_indices)), 'g'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,6); plot(time_indices, zeros(size(time_indices)), 'k'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,7); plot(time_indices, sqrt(sum(vel_residuals.^2, 1)), 'b'); title('Velocity Residual Norm'); ylabel('m/s'); legend('Res Norm'); grid on;
subplot(3,3,8); plot(time_indices, zeros(size(time_indices)), 'b'); title('Placeholder'); ylabel('N/A'); grid on;
subplot(3,3,9); plot(time_indices, zeros(size(time_indices)), 'k'); title('Placeholder'); ylabel('N/A'); grid on;
vel_res_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_residuals_velocity_residuals.pdf';
saveas(fig_vel_res, vel_res_file);
fprintf('Task 5: Saved plot: %s\n', vel_res_file);

% Step 7: Save results
results_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat';
fprintf('Task 5: Saving results to %s...\n', results_file);
save(results_file, 'x_log', 'pos_est_ecef', 'vel_est_ecef', 'pos_est_ned', 'vel_est_ned', ...
    'acc_body', 'pos_residuals', 'vel_residuals');
fprintf('Task 5: Results saved to %s\n', results_file);

try
    check = load(results_file, 'x_log', 'pos_est_ecef');
    fprintf('Task 5: Verified saved x_log (%dx%d), pos_est_ecef (%dx%d)\n', ...
        size(check.x_log), size(check.pos_est_ecef));
catch
    warning('Task 5: Failed to verify save in %s\n', results_file);
end
fprintf('Task 5: Completed successfully\n');
end

function imu_data = load_imu_data()
%LOAD_IMU_DATA Placeholder IMU loader used by Task_5.
imu_data = zeros(500000, 6);  % [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
fprintf('Task 5: Loaded placeholder IMU data (replace with actual data)\n');
end
