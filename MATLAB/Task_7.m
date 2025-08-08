function Task_7()
%TASK_7 Plot ECEF residuals between truth and fused estimate.
%   TASK_7() loads the fused state history ``x_log`` produced by Task 5 and
%   the ground truth trajectory from Task 4. Earlier versions of the
%   pipeline saved an estimator state matrix without the corresponding time
%   vector and the Task 4 MAT file omitted ``t_truth``. This function
%   reconstructs both time vectors, creates a common sampling grid and then
%   interpolates the estimator and truth trajectories prior to computing
%   residuals. All outputs are written to the ``MATLAB/results`` directory.
%
%   Usage:
%       Task_7()

    fprintf('--- Starting Task 7: Residual Analysis with Task4 truth (ECEF) ---\n');

    paths = project_paths();
    results_dir = paths.matlab_results;

    %% Load state history from Task 5
    files = dir(fullfile(results_dir, '*_task5_results.mat'));
    if isempty(files)
        error('Task_7: no Task 5 results found.');
    end
    [~, fname, ~] = fileparts(files(1).name);
    run_id = erase(fname, '_task5_results');
    mat5 = fullfile(results_dir, files(1).name);
    S5 = load(mat5);
    x_log = S5.x_log; % 15xN state log
    if isfield(S5, 't_est') && isfield(S5, 'dt')
        t_est = S5.t_est(:);
        dt    = S5.dt;
    else
        time_file = fullfile(results_dir, sprintf('%s_task5_time.mat', run_id));
        if isfile(time_file)
            Stime = load(time_file, 't_est', 'dt', 'x_log');
            if isfield(Stime, 't_est') && isfield(Stime, 'dt')
                t_est = Stime.t_est(:);
                dt    = Stime.dt;
                if ~exist('x_log', 'var') || isempty(x_log)
                    if isfield(Stime, 'x_log'); x_log = Stime.x_log; end
                end
            end
        end
        if ~(exist('t_est','var') && exist('dt','var'))
            if isfield(S5, 'x_log') && isfield(S5, 'imu_rate_hz')
                N     = size(S5.x_log, 2);
                dt    = 1 / S5.imu_rate_hz;
                t_est = (0:N-1)' * dt;
            else
                error('Task_7: estimator time vector missing and dt not provided.');
            end
        end
    end
    t_est = zero_base_time(t_est);
    ref_lat = S5.ref_lat;
    ref_lon = S5.ref_lon;
    ref_r0  = S5.ref_r0;
    fprintf('Task 7: Loaded x_log from %s, size: %dx%d\n', mat5, size(x_log));

    %% Load ground truth
    % Task 4 may not save the truth time vector. Reconstruct it from the
    % GNSS CSV if needed.
    data_dir = fileparts(fileparts(results_dir));
    mat4 = fullfile(results_dir, sprintf('Task4_results_%s.mat', run_id));
    if ~isfile(mat4)
        pair_tag = regexp(run_id, 'IMU_[^_]+_GNSS_[^_]+', 'match', 'once');
        mat4 = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
    end
    if isfile(mat4)
        S4 = load(mat4);
    else
        error('Task_7: missing Task4 results to get truth data.');
    end

    if isfield(S4, 'pos_truth')
        pos_truth_ecef = S4.pos_truth';
    elseif isfield(S4, 'truth_pos_ecef')
        pos_truth_ecef = S4.truth_pos_ecef';
    else
        error('Task_7: truth positions missing in %s.', mat4);
    end
    if isfield(S4, 'truth_vel_ecef')
        vel_truth_ecef = S4.truth_vel_ecef';
    else
        vel_truth_ecef = [];
    end

    if isfield(S4, 't_truth')
        t_truth = S4.t_truth(:);
    else
        tokens = regexp(run_id, 'GNSS_([^_]+)', 'tokens', 'once');
        csv = fullfile(data_dir, sprintf('GNSS_%s.csv', tokens{1}));
        T = readtable(csv);
        t_truth = zero_base_time(T.Posix_Time);
    end

    %% Convert estimates from NED to ECEF
    C_n_e = compute_C_ECEF_to_NED(ref_lat, ref_lon)';
    fprintf('Task 7: Extracting and converting estimates to ECEF...\n');
    pos_est_ned = x_log(1:3, :);
    vel_est_ned = x_log(4:6, :);
    pos_est_ecef = C_n_e * pos_est_ned + ref_r0;
    vel_est_ecef = C_n_e * vel_est_ned;

    %% Validate time vectors and compute common window
    if isempty(t_est) || isempty(t_truth)
        error('Task_7: time vectors empty—cannot compute residuals.');
    end
    t_start = max(min(t_est), min(t_truth));
    t_end   = min(max(t_est), max(t_truth));
    if t_end <= t_start
        error('Task_7: no overlap between estimated and truth time-series.');
    end

    %% Determine interpolation resolution using finer sample interval
    dt_est   = median(diff(t_est));
    dt_truth = median(diff(t_truth));
    dt_r = min(dt_est, dt_truth);
    t_grid = (t_start:dt_r:t_end)';

    %% Interpolate estimator and truth to common grid
    pos_est_i   = interp1(t_est,   pos_est_ecef',   t_grid, 'linear')';
    vel_est_i   = interp1(t_est,   vel_est_ecef',   t_grid, 'linear')';
    truth_pos_i = interp1(t_truth, pos_truth_ecef', t_grid, 'linear')';
    truth_vel_i = interp1(t_truth, vel_truth_ecef', t_grid, 'linear')';
    fprintf('Task 7: Interpolated estimates and truth onto %d samples\n', numel(t_grid));

    %% Compute errors (truth - estimate)
    fprintf('Task 7: Computing errors...\n');
    pos_error = truth_pos_i - pos_est_i;
    vel_error = truth_vel_i - vel_est_i;
    pos_residual = pos_est_i - truth_pos_i;
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
    fprintf('Final truth_vel_ecef: [%.8f %.8f %.8f]\n', truth_vel_i(1,end), truth_vel_i(2,end), truth_vel_i(3,end));
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
        plot(t_grid, pos_error(j,:), 'k');
        title(labels{j});
        ylabel('Position Error [m]');
        grid on;
    end
    for j = 1:3
        subplot(2,3,3+j);
        plot(t_grid, vel_error(j,:), 'k');
        ylabel('Velocity Error [m/s]');
        xlabel('Time [s]');
        grid on;
    end
    sgtitle('Truth - Estimate Errors (ECEF)');
    out_pdf = fullfile(results_dir, sprintf('%s_task7_3_residuals_position_velocity_ecef.pdf', run_id));
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
    results_out = fullfile(results_dir, sprintf('%s_task7_results.mat', run_id));
    save(results_out, 'pos_error', 'vel_error', 'pos_est_i', 'vel_est_i', 't_grid');
    fprintf('Task 7: Results saved to %s\n', results_out);
    fprintf('[SUMMARY] method=KF rmse_pos=%.2f m final_pos=%.2f m ', ...
            sqrt(mean(sum(pos_error.^2,2))), final_pos);
    fprintf('rmse_vel=%.2f m/s final_vel=%.2f m/s\n', ...
            sqrt(mean(sum(vel_error.^2,2))), final_vel);
    fprintf('Task 7: Completed successfully\n');
end
