function Task_7()
%TASK_7 Plot ECEF residuals between truth and fused estimate.
%   TASK_7() loads the fused state history ``x_log`` produced by Task 5 and
%   the ground truth trajectory.  The function first attempts to read
%   ``truth_pos_ecef`` and ``truth_vel_ecef`` from the Task 4 results MAT
%   file.  If those fields are absent (as may happen when running the
%   pipeline from scratch), the STATE_X text log is parsed using
%   ``read_state_file``.  The estimator NED states are converted to ECEF
%   using the reference latitude and longitude from Task 5.  The truth time
%   vector is synchronised to the estimator by cross-correlating position
%   and velocity magnitudes before interpolating the estimator output to
%   the aligned truth timestamps.  Position and velocity errors
%   ``truth - estimate`` are then computed.  A figure with six
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

    if isfield(d, 'truth_pos_ecef') && isfield(d, 'truth_vel_ecef') && isfield(d, 'truth_time')
        % Use truth trajectory saved by Task 4, avoiding a re-read of STATE_X log
        truth_pos_ecef = d.truth_pos_ecef';
        truth_vel_ecef = d.truth_vel_ecef';
        t_truth = d.truth_time(:);
        fprintf('Task 7: Loaded truth ECEF from %s\n', truth_file);
    else
        fprintf('Task 7: truth_pos_ecef not found in %s. Using STATE_X001.txt\n', truth_file);
        root_dir = fileparts(fileparts(results_dir));
        state_file = fullfile(root_dir, 'STATE_X001.txt');
        raw = read_state_file(state_file);
        t_truth = raw(:,2); % time column in seconds
        truth_pos_ecef = raw(:,3:5)';
        truth_vel_ecef = raw(:,6:8)';
    end

    pos_truth_ecef = truth_pos_ecef;
    vel_truth_ecef = truth_vel_ecef;

    %% Convert estimates from NED to ECEF
    C_n_e = compute_C_ECEF_to_NED(ref_lat, ref_lon)';
    fprintf('Task 7: Extracting and converting estimates to ECEF...\n');
    pos_est_ned = x_log(1:3, :);
    vel_est_ned = x_log(4:6, :);
    pos_est_ecef = C_n_e * pos_est_ned + ref_r0;
    vel_est_ecef = C_n_e * vel_est_ned;

    %% Synchronise time using position/velocity cross-correlation
    dt_r = max(mean(diff(t_est)), mean(diff(t_truth)));
    t_grid = (min([t_est(1), t_truth(1)]):dt_r:max([t_est(end), t_truth(end)]))';
    pos_est_rs = interp1(t_est, pos_est_ecef', t_grid, 'linear', 'extrap');
    pos_truth_rs = interp1(t_truth, truth_pos_ecef', t_grid, 'linear', 'extrap');
    vel_est_rs = interp1(t_est, vel_est_ecef', t_grid, 'linear', 'extrap');
    vel_truth_rs = interp1(t_truth, truth_vel_ecef', t_grid, 'linear', 'extrap');
    pos_norm_est = vecnorm(pos_est_rs, 2, 2);
    pos_norm_truth = vecnorm(pos_truth_rs, 2, 2);
    vel_norm_est = vecnorm(vel_est_rs, 2, 2);
    vel_norm_truth = vecnorm(vel_truth_rs, 2, 2);
    [xc_pos, lags_pos] = xcorr(pos_norm_est - mean(pos_norm_est), pos_norm_truth - mean(pos_norm_truth));
    [~, idx_pos] = max(xc_pos);
    [xc_vel, lags_vel] = xcorr(vel_norm_est - mean(vel_norm_est), vel_norm_truth - mean(vel_norm_truth));
    [~, idx_vel] = max(xc_vel);
    offset_pos = lags_pos(idx_pos) * dt_r;
    offset_vel = lags_vel(idx_vel) * dt_r;
    t_truth = t_truth + mean([offset_pos, offset_vel]);
    fprintf('Task 7: Applied time offset %.3f s via pos/vel alignment\n', mean([offset_pos, offset_vel]));

    %% Interpolate estimator output to aligned truth timestamps
    pos_est_i = interp1(t_est, pos_est_ecef', t_truth, 'linear', 'extrap')';
    vel_est_i = interp1(t_est, vel_est_ecef', t_truth, 'linear', 'extrap')';
    fprintf('Task 7: Interpolated estimates to %d truth samples\n', numel(t_truth));

    %% Compute errors (truth - estimate)
    fprintf('Task 7: Computing errors...\n');
    pos_error = truth_pos_ecef - pos_est_i;
    vel_error = truth_vel_ecef - vel_est_i;
    pos_residual = pos_est_i - truth_pos_ecef;
    assert(max(abs(pos_residual(:))) < 100, ...
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
    fprintf('Final truth_vel_ecef: [%.8f %.8f %.8f]\n', truth_vel_ecef(1,end), truth_vel_ecef(2,end), truth_vel_ecef(3,end));
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
