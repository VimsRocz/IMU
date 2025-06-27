

%% ========================================================================
% TASK 5: Sensor Fusion with Kalman Filter
% =========================================================================
fprintf('\nTASK 5: Sensor Fusion with Kalman Filter\n');

% Ensure Task 3 results are available
results_dir = 'results';
results_file = fullfile(results_dir, 'task3_results.mat');
if ~isfile(results_file)
    error('Task_5:MissingFile', 'Task 3 results not found: %s', results_file);
end

%% ========================================================================
% Subtask 5.1-5.5: Configure and Initialize 9-State Filter
% =========================================================================
fprintf('\nSubtask 5.1-5.5: Configuring and Initializing 9-State Kalman Filter.\n');
% State vector x: [pos; vel; acc]' (9x1)
x = zeros(9, 1);
x(1:3) = gnss_pos_ned(1,:)';
x(4:6) = gnss_vel_ned(1,:)';
% Initial acceleration is assumed to be zero
P = eye(9) * 1.0; % Covariance matrix (9x9)
Q = eye(9) * 0.01; % Process noise covariance (9x9)
R = eye(6) * 0.1;  % Measurement noise covariance (6x6 for pos & vel)
H = [eye(6), zeros(6,3)]; % Measurement matrix (6x9)

% --- Attitude Initialization ---
q_b_n = rot_to_quaternion(C_B_N); % Initial attitude quaternion

% --- Pre-allocate Log Arrays ---
num_imu_samples = length(imu_time);
x_log = zeros(9, num_imu_samples);
euler_log = zeros(3, num_imu_samples);
zupt_log = zeros(1, num_imu_samples);
zupt_count = 0;
fprintf('-> 9-State filter initialized.\n');

%% ========================================================================
% Subtask 5.6: Kalman Filter for Sensor Fusion
% =========================================================================
fprintf('\nSubtask 5.6: Running Kalman Filter for sensor fusion.\n');

% Interpolate GNSS measurements to IMU timestamps
gnss_pos_interp = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear', 'extrap');
gnss_vel_interp = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear', 'extrap');

% --- Main Filter Loop ---
fprintf('-> Starting filter loop over %d IMU samples...\n', num_imu_samples);
for i = 1:num_imu_samples
    % --- 1. State Propagation (Prediction) ---
    F = eye(9);
    F(1:3, 4:6) = eye(3) * dt_imu;
    F(4:6, 7:9) = eye(3) * dt_imu;
    
    x = F * x;
    P = F * P * F' + Q * dt_imu;
    
    % --- 2. Attitude Propagation ---
    w_b = gyro_body_raw(i,:)'; % Using raw gyro for propagation
    q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
    
    % --- 3. Measurement Update (Correction) ---
    z = [gnss_pos_interp(i,:)'; gnss_vel_interp(i,:)'];
    y = z - H * x;
    S = H * P * H' + R;
    K = (P * H') / S;
    x = x + K * y;
    P = (eye(9) - K * H) * P;
    
    % --- 4. Zero-Velocity Update (ZUPT) ---
    win_size = 80;
    if i > win_size
        acc_win = acc_body_raw(i-win_size+1:i, :);
        gyro_win = gyro_body_raw(i-win_size+1:i, :);
        if is_static(acc_win, gyro_win)
            zupt_count = zupt_count + 1;
            zupt_log(i) = 1;
            H_z = [zeros(3), eye(3), zeros(3)]; % Measurement model for velocity in 9-state vector
            R_z = eye(3) * 1e-4;
            y_z = zeros(3, 1) - H_z * x;
            S_z = H_z * P * H_z' + R_z;
            K_z = (P * H_z') / S_z;
            x = x + K_z * y_z;
            P = (eye(9) - K_z * H_z) * P;
        end
    end

    % --- Log State and Attitude ---
    x_log(:, i) = x;
    euler_log(:, i) = quat_to_euler(q_b_n);
end
fprintf('-> Filter loop complete. Total ZUPT applications: %d\n', zupt_count);

%% ========================================================================
% Subtask 5.7: Handle Event at 5000s
% =========================================================================
fprintf('\nSubtask 5.7: No event handling needed as time < 5000s.\n');

%% ========================================================================
% Subtask 5.8: Plotting Results
% =========================================================================
fprintf('\nSubtask 5.8: Plotting Kalman filter results.\n');

% --- Plot 1: Position Comparison ---
figure('Name', 'KF Results: Position', 'Position', [100 100 1200 600]);
labels = {'North', 'East', 'Down'};
for i = 1:3
    subplot(3, 1, i); hold on;
    plot(gnss_time, gnss_pos_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Raw)');
    plot(imu_time, x_log(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    hold off; grid on; legend; ylabel('[m]'); title(['Position: ' labels{i}]);
end
xlabel('Time (s)'); sgtitle('Kalman Filter Fused Position vs. GNSS');
saveas(gcf, fullfile(results_dir, [tag '_position.pdf']));
fprintf('-> Position plot saved.\n');

% --- Plot 2: Velocity Comparison ---
figure('Name', 'KF Results: Velocity', 'Position', [150 150 1200 600]);
for i = 1:3
    subplot(3, 1, i); hold on;
    plot(gnss_time, gnss_vel_ned(:,i), 'k:', 'LineWidth', 1, 'DisplayName', 'GNSS (Raw)');
    plot(imu_time, x_log(i+3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused (KF)');
    zupt_indices = find(zupt_log);
    if ~isempty(zupt_indices), plot(imu_time(zupt_indices), x_log(i+3,zupt_indices), 'ro', 'MarkerSize', 3, 'DisplayName', 'ZUPT'); end
    hold off; grid on; legend; ylabel('[m/s]'); title(['Velocity: ' labels{i}]);
end
xlabel('Time (s)'); sgtitle('Kalman Filter Fused Velocity vs. GNSS');
saveas(gcf, fullfile(results_dir, [tag '_velocity.pdf']));
fprintf('-> Velocity plot saved.\n');

% --- Plot 3: Attitude (Euler Angles) ---
figure('Name', 'KF Results: Attitude', 'Position', [200 200 1200 600]);
euler_labels = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3, 1, i);
    plot(imu_time, rad2deg(euler_log(i,:)), 'b-');
    grid on; ylabel('[deg]'); title([euler_labels{i} ' Angle']);
end
xlabel('Time (s)'); sgtitle('Attitude Estimate Over Time');
saveas(gcf, fullfile(results_dir, [tag '_attitude.pdf']));
fprintf('-> Attitude plot saved.\n');

end % End of main function

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================
function q_new = propagate_quaternion(q_old, w, dt)
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
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    q_out = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
             w1*x2 + x1*w2 + y1*z2 - z1*y2;
             w1*y2 - x1*z2 + y1*w2 + z1*x2;
             w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function euler = quat_to_euler(q)
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
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1 - 2 * (qy^2 + qz^2), 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy);
         2 * (qx*qy + qw*qz), 1 - 2 * (qx^2 + qz^2), 2 * (qy*qz - qw*qx);
         2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 1 - 2 * (qx^2 + qy^2)];
end

function q = rot_to_quaternion(R)
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
function is_stat=is_static(acc,gyro)
    acc_thresh=0.01;gyro_thresh=1e-6;
    is_stat=all(var(acc,0,1)<acc_thresh)&&all(var(gyro,0,1)<gyro_thresh);
end

