function Task_6()
%TASK_6  Generate 3x3 overlay plot in the ECEF frame using STATE_X001.txt
%   This simplified version loads the fused state history from Task 5 and
%   the truth trajectory from ``STATE_X001.txt``. The fused NED estimates
%   are converted to ECEF coordinates using the rotation matrix from Task 4.
%   Position and velocity components are plotted against truth in a 3x3
%   grid (XYZ and VX,VY,VZ) along with norms and a position error magnitude.
%   Results are saved to ``results/``. File paths are currently hard-coded
%   for demonstration purposes.
%
%   Usage:
%       Task_6()

    fprintf('--- Starting Task 6: Overlay Plots with STATE_X001.txt (ECEF) ---\n');

    % Load state history
    results_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat';
    try
        load(results_file, 'x_log');
        fprintf('Task 6: Loaded x_log from %s, size: %dx%d\n', results_file, size(x_log));
    catch
        error('Task 6: Failed to load x_log from %s.', results_file);
    end

    % Load truth data
    truth_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/STATE_X001.txt';
    try
        truth_data = readmatrix(truth_file);
        fprintf('Task 6: Loaded truth data from %s, size: %dx%d\n', truth_file, size(truth_data));
    catch
        error('Task 6: Failed to load truth data from %s.', truth_file);
    end

    % Extract truth position and velocity
    pos_truth_ecef = truth_data(:, 2:4)';
    vel_truth_ecef = truth_data(:, 5:7)';

    % Load NED-to-ECEF rotation matrix
    task4_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/Task4_results_IMU_X002_GNSS_X002.mat';
    try
        load(task4_file, 'C_n_e');
        fprintf('Task 6: Loaded C_n_e from %s\n', task4_file);
    catch
        error('Task 6: Failed to load C_n_e from %s.', task4_file);
    end

    % Extract and convert estimates
    fprintf('Task 6: Extracting and converting estimates to ECEF...\n');
    pos_est_ned = x_log(1:3, :);  % NED position
    vel_est_ned = x_log(4:6, :);  % NED velocity
    pos_est_ecef = C_n_e * pos_est_ned;
    vel_est_ecef = C_n_e * vel_est_ned;

    % Downsample estimates
    downsample_factor = 400;  % 500,000 / 1250
    time_indices = 1:downsample_factor:500000;
    pos_est_ecef = pos_est_ecef(:, time_indices);
    vel_est_ecef = vel_est_ecef(:, time_indices);
    fprintf('Task 6: Downsampled estimates to %d samples (factor: %d)\n', length(time_indices), downsample_factor);

    % Validate data lengths
    if size(pos_truth_ecef, 2) ~= length(time_indices)
        error('Task 6: Data length mismatch. Truth: %d, Estimated: %d.', size(pos_truth_ecef, 2), length(time_indices));
    end
    fprintf('Task 6: Validated data lengths: %d samples\n', length(time_indices));

    % Print state values
    fprintf('Subtask 6.8.2: Plotted TRIAD position X_ECEF: First = %.4f, Last = %.4f m\n', pos_est_ecef(1,1), pos_est_ecef(1,end));
    fprintf('Subtask 6.8.2: Plotted TRIAD position Y_ECEF: First = %.4f, Last = %.4f m\n', pos_est_ecef(2,1), pos_est_ecef(2,end));
    fprintf('Subtask 6.8.2: Plotted TRIAD position Z_ECEF: First = %.4f, Last = %.4f m\n', pos_est_ecef(3,1), pos_est_ecef(3,end));
    fprintf('Subtask 6.8.2: Plotted TRIAD velocity X_ECEF: First = %.4f, Last = %.4f m/s\n', vel_est_ecef(1,1), vel_est_ecef(1,end));
    fprintf('Subtask 6.8.2: Plotted TRIAD velocity Y_ECEF: First = %.4f, Last = %.4f m/s\n', vel_est_ecef(2,1), vel_est_ecef(2,end));
    fprintf('Subtask 6.8.2: Plotted TRIAD velocity Z_ECEF: First = %.4f, Last = %.4f m/s\n', vel_est_ecef(3,1), vel_est_ecef(3,end));

    % Generate 3x3 subplot overlay plot
    fprintf('Task 6: Generating and displaying 3x3 ECEF overlay plot...\n');
    fig = figure('Name', 'Task 6 - ECEF State Overlay (3x3)', 'Visible', 'on');

    % Position X
    subplot(3,3,1);
    plot(time_indices, pos_est_ecef(1,:), 'b', 'DisplayName', 'Est X');
    hold on;
    plot(time_indices, pos_truth_ecef(1,:), 'r--', 'DisplayName', 'Truth X');
    title('Position X (ECEF)');
    xlabel('Time Step'); ylabel('Position (m)');
    legend('Location', 'best');
    grid on;

    % Position Y
    subplot(3,3,2);
    plot(time_indices, pos_est_ecef(2,:), 'g', 'DisplayName', 'Est Y');
    hold on;
    plot(time_indices, pos_truth_ecef(2,:), 'm--', 'DisplayName', 'Truth Y');
    title('Position Y (ECEF)');
    xlabel('Time Step'); ylabel('Position (m)');
    legend('Location', 'best');
    grid on;

    % Position Z
    subplot(3,3,3);
    plot(time_indices, pos_est_ecef(3,:), 'k', 'DisplayName', 'Est Z');
    hold on;
    plot(time_indices, pos_truth_ecef(3,:), 'c--', 'DisplayName', 'Truth Z');
    title('Position Z (ECEF)');
    xlabel('Time Step'); ylabel('Position (m)');
    legend('Location', 'best');
    grid on;

    % Velocity X
    subplot(3,3,4);
    plot(time_indices, vel_est_ecef(1,:), 'b', 'DisplayName', 'Est VX');
    hold on;
    plot(time_indices, vel_truth_ecef(1,:), 'r--', 'DisplayName', 'Truth VX');
    title('Velocity X (ECEF)');
    xlabel('Time Step'); ylabel('Velocity (m/s)');
    legend('Location', 'best');
    grid on;

    % Velocity Y
    subplot(3,3,5);
    plot(time_indices, vel_est_ecef(2,:), 'g', 'DisplayName', 'Est VY');
    hold on;
    plot(time_indices, vel_truth_ecef(2,:), 'm--', 'DisplayName', 'Truth VY');
    title('Velocity Y (ECEF)');
    xlabel('Time Step'); ylabel('Velocity (m/s)');
    legend('Location', 'best');
    grid on;

    % Velocity Z
    subplot(3,3,6);
    plot(time_indices, vel_est_ecef(3,:), 'k', 'DisplayName', 'Est VZ');
    hold on;
    plot(time_indices, vel_truth_ecef(3,:), 'c--', 'DisplayName', 'Truth VZ');
    title('Velocity Z (ECEF)');
    xlabel('Time Step'); ylabel('Velocity (m/s)');
    legend('Location', 'best');
    grid on;

    % Position Norm
    subplot(3,3,7);
    pos_norm_est = sqrt(sum(pos_est_ecef.^2, 1));
    pos_norm_truth = sqrt(sum(pos_truth_ecef.^2, 1));
    plot(time_indices, pos_norm_est, 'b', 'DisplayName', 'Est Norm');
    hold on;
    plot(time_indices, pos_norm_truth, 'r--', 'DisplayName', 'Truth Norm');
    title('Position Norm (ECEF)');
    xlabel('Time Step'); ylabel('Norm (m)');
    legend('Location', 'best');
    grid on;

    % Velocity Norm
    subplot(3,3,8);
    vel_norm_est = sqrt(sum(vel_est_ecef.^2, 1));
    vel_norm_truth = sqrt(sum(vel_truth_ecef.^2, 1));
    plot(time_indices, vel_norm_est, 'b', 'DisplayName', 'Est Norm');
    hold on;
    plot(time_indices, vel_norm_truth, 'r--', 'DisplayName', 'Truth Norm');
    title('Velocity Norm (ECEF)');
    xlabel('Time Step'); ylabel('Norm (m/s)');
    legend('Location', 'best');
    grid on;

    % Position Error Norm
    subplot(3,3,9);
    error_norm = sqrt(sum((pos_est_ecef - pos_truth_ecef).^2, 1));
    plot(time_indices, error_norm, 'k', 'DisplayName', 'Position Error Norm');
    title('Position Error Norm (ECEF)');
    xlabel('Time Step'); ylabel('Error (m)');
    legend('Location', 'best');
    grid on;

    % Save plot
    output_file = '/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task6_overlay_state_ECEF.pdf';
    saveas(fig, output_file);
    fprintf('Task 6: Saved overlay figure: %s\n', output_file);

    % Save results
    save('/Users/vimalchawda/Desktop/IMU/MATLAB/results/IMU_X002_GNSS_X002_TRIAD_task6_results.mat', ...
        'pos_est_ecef', 'vel_est_ecef', 'pos_truth_ecef', 'vel_truth_ecef');
    fprintf('Task 6: Results saved to results/IMU_X002_GNSS_X002_TRIAD_task6_results.mat\n');
    fprintf('Task 6: Completed successfully\n');
end
