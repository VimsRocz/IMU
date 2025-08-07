function Task_7()
%TASK_7 Plot ECEF residuals between truth and fused estimate.
%   TASK_7() loads the fused state history ``x_log`` produced by Task 5 and
%   the ground truth trajectory.  The function first attempts to read
%   ``truth_pos_ecef`` and ``truth_vel_ecef`` from the Task 4 results MAT
%   file.  If those fields are absent (as may happen when running the
%   pipeline from scratch), the STATE_X text log is parsed using
%   ``read_state_file``.  The estimator NED states are converted to ECEF
%   using the reference latitude and longitude from Task 5.  Position and
%   velocity errors ``truth - estimate`` are computed after interpolating
%   the estimator output to the truth time vector.  A figure with six
%   subplots (position X/Y/Z on the first row, velocity X/Y/Z on the second)
%   is generated and saved under ``results``.

    fprintf('--- Starting Task 7: Residual Analysis with Task4 truth (ECEF) ---\n');

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

    %% Load ground truth
    % Prefer the Task 4 results file which may contain the truth ECEF
    % trajectory. If not available, fall back to the STATE_X text log as in
    % Task 6.
    truth_file = fullfile(results_dir, 'Task4_results_IMU_X002_GNSS_X002.mat');
    if isfile(truth_file)
        d = load(truth_file);
    else
        d = struct();
    end

    if isfield(d, 'truth_pos_ecef') && isfield(d, 'truth_vel_ecef')
        truth_pos_ecef = d.truth_pos_ecef';
        truth_vel_ecef = d.truth_vel_ecef';
        t_truth = (0:size(truth_pos_ecef,2)-1)';
        pos_truth_ecef = truth_pos_ecef;
        vel_truth_ecef = truth_vel_ecef;
        fprintf('Task 7: Loaded truth ECEF from %s\n', truth_file);
    else
        fprintf('Task 7: truth_pos_ecef not found in %s. Using STATE_X001.txt\n', truth_file);
        root_dir = fileparts(fileparts(results_dir));
        state_file = fullfile(root_dir, 'STATE_X001.txt');
        raw = read_state_file(state_file);
        t_truth = raw(:,2); % time column in seconds
        truth_pos_ecef = raw(:,3:5)';
        truth_vel_ecef = raw(:,6:8)';
        pos_truth_ecef = truth_pos_ecef;
        vel_truth_ecef = truth_vel_ecef;
    end

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

    %% Compute errors (truth - estimate)
    fprintf('Task 7: Computing errors...\n');
    pos_error = pos_truth_ecef - pos_est_i;
    vel_error = vel_truth_ecef - vel_est_i;
    pos_residual = pos_est_i - pos_truth_ecef;
    assert(max(abs(pos_residual), [], 'all') < 100, ...
        'Task-7: Position residual blew up - transform error?');

    final_pos = norm(pos_error(:,end));
    final_vel = norm(vel_error(:,end));

    %% Error statistics
    pos_err_mean = mean(pos_error, 2);
    pos_err_std  = std(pos_error, 0, 2);
    vel_err_mean = mean(vel_error, 2);
    vel_err_std  = std(vel_error, 0, 2);
    fprintf('Position error mean [m]: [%.8f %.8f %.8f]\n', pos_err_mean(1), pos_err_mean(2), pos_err_mean(3));
    fprintf('Position error std  [m]: [%.8f %.8f %.8f]\n', pos_err_std(1), pos_err_std(2), pos_err_std(3));
    fprintf('Velocity error mean [m/s]: [%.8f %.8f %.8f]\n', vel_err_mean(1), vel_err_mean(2), vel_err_mean(3));
    fprintf('Velocity error std  [m/s]: [%.8f %.8f %.8f]\n', vel_err_std(1), vel_err_std(2), vel_err_std(3));

    fprintf('Final fused_vel_ecef: [%.8f %.8f %.8f]\n', vel_est_i(1,end), vel_est_i(2,end), vel_est_i(3,end));
    fprintf('Final truth_vel_ecef: [%.8f %.8f %.8f]\n', vel_truth_ecef(1,end), vel_truth_ecef(2,end), vel_truth_ecef(3,end));
    fprintf('[SUMMARY] method=KF rmse_pos=%.2f m final_pos=%.2f m ', ...
            sqrt(mean(sum(pos_error.^2,2))), final_pos);
    fprintf('rmse_vel=%.2f m/s final_vel=%.2f m/s\n', ...
            sqrt(mean(sum(vel_error.^2,2))), final_vel);

    %% Plot errors
    fprintf('Task 7: Generating ECEF error plots...\n');
    fig = figure('Name', 'Task 7 - ECEF Errors', 'Visible', 'on', 'Position',[100 100 900 450]);
    labels = {'X','Y','Z'};
    for j = 1:3
        subplot(2,3,j);
        plot(t_truth, pos_error(j,:), 'k');
        title(labels{j});
        ylabel('Position Error [m]');
        grid on;
    end
    for j = 1:3
        subplot(2,3,3+j);
        plot(t_truth, vel_error(j,:), 'k');
        ylabel('Velocity Error [m/s]');
        xlabel('Time [s]');
        grid on;
    end
    sgtitle('Truth - Estimate Errors (ECEF)');
    out_pdf = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task7_3_residuals_position_velocity_ecef.pdf');
    saveas(fig, out_pdf);
    fprintf('Task 7: Saved error plot: %s\n', out_pdf);

    %% Difference ranges
    fprintf('Task 7: Computing difference ranges...\n');
    directions = {'X','Y','Z'};
    for i = 1:3
        pos_range = [min(pos_error(i,:)) max(pos_error(i,:))];
        vel_range = [min(vel_error(i,:)) max(vel_error(i,:))];
        pos_exceed = sum(abs(pos_error(i,:)) > 1);
        vel_exceed = sum(abs(vel_error(i,:)) > 1);
        fprintf('ECEF %s position error range: %.2f m to %.2f m. %d samples exceed 1.0 m\n', ...
            directions{i}, pos_range(1), pos_range(2), pos_exceed);
        fprintf('ECEF %s velocity error range: %.2f m/s to %.2f m/s. %d samples exceed 1.0 m/s\n', ...
            directions{i}, vel_range(1), vel_range(2), vel_exceed);
    end

    %% Save results
    results_out = fullfile(results_dir, 'IMU_X002_GNSS_X002_TRIAD_task7_results.mat');
    save(results_out, 'pos_error', 'vel_error', 'pos_est_i', 'vel_est_i');
    fprintf('Task 7: Results saved to %s\n', results_out);
    fprintf('[SUMMARY] method=KF rmse_pos=%.2f m final_pos=%.2f m ', ...
            sqrt(mean(sum(pos_error.^2,2))), final_pos);
    fprintf('rmse_vel=%.2f m/s final_vel=%.2f m/s\n', ...
            sqrt(mean(sum(vel_error.^2,2))), final_vel);
    fprintf('Task 7: Completed successfully\n');
end