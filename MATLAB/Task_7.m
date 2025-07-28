function Task_7()
%TASK_7 Residual analysis against STATE\_X001.txt in the ECEF frame.
%   TASK_7() loads the fused state history ``x_log`` saved by Task 5 and
%   the ground truth trajectory ``STATE_X001.txt``.  The estimator NED
%   states are converted to the ECEF frame using the reference latitude and
%   longitude from Task 5.  Residuals in position and velocity are
%   computed after interpolating the estimator output to the truth time
%   vector.  Summary statistics are printed and residual plots are shown
%   interactively.  Results are written to the ``results`` directory.

    fprintf('--- Starting Task 7: Residual Analysis with STATE_X001.txt (ECEF) ---\n');

    %% Load state history from Task 5
    results_dir = get_results_dir();
    res_file = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task5_results.mat');
    try
        S = load(res_file, 'x_log', 'time', 'ref_lat', 'ref_lon', 'ref_r0');
        x_log  = S.x_log;
        t_est  = S.time(:);
        ref_lat = S.ref_lat;
        ref_lon = S.ref_lon;
        ref_r0  = S.ref_r0;
        fprintf('Task 7: Loaded x_log from %s, size: %dx%d\n', res_file, size(x_log));
    catch ME
        error('Task 7: Failed to load x_log from %s. %s', res_file, ME.message);
    end

    %% Load ground truth STATE_X001.txt
    root_dir = fileparts(fileparts(mfilename('fullpath')));
    truth_path = fullfile(root_dir, 'STATE_X001.txt');
    try
        truth_data = read_state_file(truth_path);
        fprintf('Task 7: Loaded truth data from %s, size: %dx%d\n', truth_path, size(truth_data));
    catch ME
        error('Task 7: Failed to load truth data from %s. %s', truth_path, ME.message);
    end

    %% Extract truth position and velocity
    t_truth = truth_data(:,2);
    pos_truth_ecef = truth_data(:,3:5)';
    vel_truth_ecef = truth_data(:,6:8)';

    %% Convert estimates from NED to ECEF
    C_n_e = compute_C_ECEF_to_NED(ref_lat, ref_lon)';
    fprintf('Task 7: Extracting and converting estimates to ECEF...\n');
    pos_est_ned = x_log(1:3, :);
    vel_est_ned = x_log(4:6, :);
    pos_est_ecef = C_n_e * pos_est_ned + ref_r0;
    vel_est_ecef = C_n_e * vel_est_ned;

    %% Interpolate estimator output to truth timestamps
    pos_est_i = interp1(t_est, pos_est_ecef', t_truth, 'linear', 'extrap')';
    vel_est_i = interp1(t_est, vel_est_ecef', t_truth, 'linear', 'extrap')';
    fprintf('Task 7: Interpolated estimates to %d truth samples\n', numel(t_truth));

    %% Compute residuals
    fprintf('Task 7: Computing residuals...\n');
    pos_residuals = pos_est_i - pos_truth_ecef;
    vel_residuals = vel_est_i - vel_truth_ecef;

    %% Residual statistics
    pos_res_mean = mean(pos_residuals, 2);
    pos_res_std  = std(pos_residuals, 0, 2);
    vel_res_mean = mean(vel_residuals, 2);
    vel_res_std  = std(vel_residuals, 0, 2);
    fprintf('Position residual mean [m]: [%.8f %.8f %.8f]\n', pos_res_mean(1), pos_res_mean(2), pos_res_mean(3));
    fprintf('Position residual std  [m]: [%.8f %.8f %.8f]\n', pos_res_std(1), pos_res_std(2), pos_res_std(3));
    fprintf('Velocity residual mean [m/s]: [%.8f %.8f %.8f]\n', vel_res_mean(1), vel_res_mean(2), vel_res_mean(3));
    fprintf('Velocity residual std  [m/s]: [%.8f %.8f %.8f]\n', vel_res_std(1), vel_res_std(2), vel_res_std(3));

    fprintf('Final fused_vel_ecef: [%.8f %.8f %.8f]\n', vel_est_i(1,end), vel_est_i(2,end), vel_est_i(3,end));
    fprintf('Final truth_vel_ecef: [%.8f %.8f %.8f]\n', vel_truth_ecef(1,end), vel_truth_ecef(2,end), vel_truth_ecef(3,end));

    %% Plot residuals
    fprintf('Task 7: Generating and displaying ECEF residual plots...\n');
    fig = figure('Name', 'Task 7 - ECEF Residuals', 'Visible', 'on');
    subplot(2,1,1);
    plot(t_truth, pos_residuals(1,:), 'b', 'DisplayName','X'); hold on;
    plot(t_truth, pos_residuals(2,:), 'g', 'DisplayName','Y');
    plot(t_truth, pos_residuals(3,:), 'k', 'DisplayName','Z');
    title('Position Residuals (ECEF)'); xlabel('Time [s]'); ylabel('Residual (m)');
    legend('Location','best'); grid on; hold off;

    subplot(2,1,2);
    plot(t_truth, vel_residuals(1,:), 'b', 'DisplayName','VX'); hold on;
    plot(t_truth, vel_residuals(2,:), 'g', 'DisplayName','VY');
    plot(t_truth, vel_residuals(3,:), 'k', 'DisplayName','VZ');
    title('Velocity Residuals (ECEF)'); xlabel('Time [s]'); ylabel('Residual (m/s)');
    legend('Location','best'); grid on; hold off;

    out_pdf = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task7_3_residuals_position_velocity_ecef.pdf');
    saveas(fig, out_pdf);
    fprintf('Task 7: Saved residual plot: %s\n', out_pdf);

    %% Difference ranges
    fprintf('Task 7: Computing difference ranges...\n');
    directions = {'X','Y','Z'};
    for i=1:3
        pos_range = [min(pos_residuals(i,:)) max(pos_residuals(i,:))];
        vel_range = [min(vel_residuals(i,:)) max(vel_residuals(i,:))];
        pos_exceed = sum(abs(pos_residuals(i,:)) > 1);
        vel_exceed = sum(abs(vel_residuals(i,:)) > 1);
        fprintf('ECEF %s position diff range: %.2f m to %.2f m. %d samples exceed 1.0 m\n', ...
            directions{i}, pos_range(1), pos_range(2), pos_exceed);
        fprintf('ECEF %s velocity diff range: %.2f m/s to %.2f m/s. %d samples exceed 1.0 m/s\n', ...
            directions{i}, vel_range(1), vel_range(2), vel_exceed);
    end

    %% Save results
    results_out = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task7_results.mat');
    save(results_out, 'pos_residuals', 'vel_residuals', 'pos_est_i', 'vel_est_i');
    fprintf('Task 7: Results saved to %s\n', results_out);
    fprintf('Task 7: Completed successfully\n');
end
