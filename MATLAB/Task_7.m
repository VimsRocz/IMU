function Task_7()
%TASK_7  Residual analysis for the TRIAD fused solution.
%   TASK_7() loads ``x_log`` from Task 5 and the GNSS NED truth data from
%   Task 4 for dataset X002. Position and velocity residuals are computed
%   after downsampling the estimator output to match the 1 Hz GNSS rate.
%   Statistics are printed to the command window and residual plots are
%   displayed interactively. Figures and MAT files are saved under the
%   ``results`` directory. This mirrors ``task7_ned_residuals_plot.py``.

    fprintf('--- Starting Task 7: Residual Analysis for IMU_X002_GNSS_X002_TRIAD ---\n');

    results_dir = get_results_dir();
    results_file = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task5_results.mat');
    try
        load(results_file, 'x_log');
        fprintf('Task 7: Loaded x_log from %s, size: %dx%d\n', results_file, size(x_log));
    catch
        error('Task 7: Failed to load x_log from %s. Ensure Task 5 saved it correctly.', results_file);
    end

    truth_file = fullfile(results_dir, 'Task4_results_IMU_X002_GNSS_X002.mat');
    try
        load(truth_file, 'gnss_pos_ned');
        fprintf('Task 7: Loaded GNSS truth positions from %s, size: %dx%d\n', truth_file, size(gnss_pos_ned));
    catch
        error('Task 7: Failed to load GNSS truth data from %s.', truth_file);
    end

    fprintf('Task 7: Extracting position and velocity estimates from x_log...\n');
    pos_est = x_log(1:3, :);  % NED position [North, East, Down]
    vel_est = x_log(4:6, :);  % NED velocity [North, East, Down]
    pos_ned_gnss = gnss_pos_ned';

    dt = 1;  % GNSS sample interval (seconds)
    vel_ned_gnss = diff(gnss_pos_ned) / dt;
    vel_ned_gnss = [vel_ned_gnss; vel_ned_gnss(end,:)]';
    fprintf('Task 7: Computed GNSS velocity, size: %dx%d\n', size(vel_ned_gnss));

    downsample_factor = 400;  % 500000 IMU samples / 1250 GNSS samples
    time_indices = 1:downsample_factor:500000;
    pos_est = pos_est(:, time_indices);
    vel_est = vel_est(:, time_indices);
    fprintf('Task 7: Downsampled estimates to %d samples (factor: %d)\n', length(time_indices), downsample_factor);

    if size(pos_ned_gnss, 2) ~= length(time_indices)
        error('Task 7: Data length mismatch. Truth: %d, Estimated: %d.', size(pos_ned_gnss, 2), length(time_indices));
    end
    fprintf('Task 7: Validated data lengths: %d samples\n', length(time_indices));

    fprintf('Task 7: Computing residuals...\n');
    pos_residuals = pos_est - pos_ned_gnss;
    vel_residuals = vel_est - vel_ned_gnss;

    pos_res_mean = mean(pos_residuals, 2);
    pos_res_std  = std(pos_residuals, 0, 2);
    vel_res_mean = mean(vel_residuals, 2);
    vel_res_std  = std(vel_residuals, 0, 2);
    fprintf('Position residual mean [m]: [%.8f %.8f %.8f]\n', pos_res_mean(1), pos_res_mean(2), pos_res_mean(3));
    fprintf('Position residual std  [m]: [%.8f %.8f %.8f]\n', pos_res_std(1), pos_res_std(2), pos_res_std(3));
    fprintf('Velocity residual mean [m/s]: [%.8f %.8f %.8f]\n', vel_res_mean(1), vel_res_mean(2), vel_res_mean(3));
    fprintf('Velocity residual std  [m/s]: [%.8f %.8f %.8f]\n', vel_res_std(1), vel_res_std(2), vel_res_std(3));

    fprintf('Final fused_vel_ned: [%.8f %.8f %.8f]\n', vel_est(1,end), vel_est(2,end), vel_est(3,end));
    fprintf('Final truth_vel_ned: [%.8f %.8f %.8f]\n', vel_ned_gnss(1,end), vel_ned_gnss(2,end), vel_ned_gnss(3,end));

    fprintf('Task 7: Generating and displaying residual plots...\n');
    fig = figure('Name', 'Task 7 - Residuals', 'Visible', 'on');
    subplot(2,1,1);
    plot(time_indices, pos_residuals(1,:), 'b', 'DisplayName', 'North'); hold on;
    plot(time_indices, pos_residuals(2,:), 'g', 'DisplayName', 'East');
    plot(time_indices, pos_residuals(3,:), 'k', 'DisplayName', 'Down');
    title('Position Residuals (NED)'); xlabel('Time Step'); ylabel('Residual (m)');
    legend('Location', 'best'); grid on; hold off;

    subplot(2,1,2);
    plot(time_indices, vel_residuals(1,:), 'b', 'DisplayName', 'North'); hold on;
    plot(time_indices, vel_residuals(2,:), 'g', 'DisplayName', 'East');
    plot(time_indices, vel_residuals(3,:), 'k', 'DisplayName', 'Down');
    title('Velocity Residuals (NED)'); xlabel('Time Step'); ylabel('Residual (m/s)');
    legend('Location', 'best'); grid on; hold off;

    out_pdf = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task7_3_residuals_position_velocity.pdf');
    saveas(fig, out_pdf);
    fprintf('Task 7: Saved residual plot: %s\n', out_pdf);

    fprintf('Task 7: Computing difference ranges...\n');
    directions = {'North', 'East', 'Down'};
    for i = 1:3
        pos_range = [min(pos_residuals(i,:)) max(pos_residuals(i,:))];
        vel_range = [min(vel_residuals(i,:)) max(vel_residuals(i,:))];
        pos_exceed = sum(abs(pos_residuals(i,:)) > 1);
        vel_exceed = sum(abs(vel_residuals(i,:)) > 1);
        fprintf('NED %s position diff range: %.2f m to %.2f m. %d samples exceed 1.0 m\n', ...
            directions{i}, pos_range(1), pos_range(2), pos_exceed);
        fprintf('NED %s velocity diff range: %.2f m/s to %.2f m/s. %d samples exceed 1.0 m/s\n', ...
            directions{i}, vel_range(1), vel_range(2), vel_exceed);
    end

    results_out = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task7_results.mat');
    save(results_out, 'pos_residuals', 'vel_residuals', 'pos_ned_gnss', 'vel_ned_gnss');
    fprintf('Task 7: Results saved to %s\n', results_out);
    fprintf('Task 7: Completed successfully\n');
end
