function summary = Task_7(tag)
%TASK_7  Evaluate fused results against GNSS truth (Task 7).
%   SUMMARY = TASK_7(TAG) loads the overlay data produced by Task 6 for
%   the run identified by TAG and computes position, velocity and
%   acceleration differences between the fused estimate and the truth in
%   the NED, ECEF and Body frames. Basic statistics are printed for each
%   axis and residual figures are saved under ``results/`` using the same
%   naming convention as the Python implementation. If the Task 5 results
%   file contains attitude history, Euler angles are also plotted. A
%   structure of summary metrics is returned and written to
%   ``results/<tag>_task7_summary.mat``.
%
%   Example:
%       summary = Task_7('IMU_X002_GNSS_X002_TRIAD');
%
%   This function mirrors ``evaluate_filter_results.py`` from the Python
%   pipeline.

if nargin < 1 || isempty(tag)
    error('Task_7:TagRequired', 'Dataset tag is required');
end

results_dir = get_results_dir();
overlay_file = fullfile(results_dir, sprintf('%s_task6_overlay.mat', tag));
if ~isfile(overlay_file)
    error('Task_7:OverlayMissing', 'Missing overlay file: %s', overlay_file);
end
L = load(overlay_file);

% Attempt to load Euler angles from the Task 5 result for attitude plots
att_deg = [];
task5_file = fullfile(results_dir, sprintf('%s_task5_results.mat', tag));
if isfile(task5_file)
    S5 = load(task5_file, 'euler_log', 'time');
    if isfield(S5, 'euler_log')
        t_att = S5.time(:) - S5.time(1);
        att_deg = rad2deg(S5.euler_log');
        if numel(t_att) ~= numel(L.tt)
            att_deg = interp1(t_att, att_deg, L.tt(:), 'linear', 'extrap');
        end
    end
end

t_start = tic;
frames = {'NED', 'ECEF', 'Body'};
labels = {{'North','East','Down'}, {'X','Y','Z'}, {'X','Y','Z'}};
summary = struct();

for f = 1:numel(frames)
    frame = frames{f};
    fn_pos_t = sprintf('pos_truth_%s', lower(frame));
    fn_vel_t = sprintf('vel_truth_%s', lower(frame));
    fn_acc_t = sprintf('acc_truth_%s', lower(frame));
    fn_pos_e = sprintf('pos_est_%s',  lower(frame));
    fn_vel_e = sprintf('vel_est_%s',  lower(frame));
    fn_acc_e = sprintf('acc_est_%s',  lower(frame));

    pos_err = L.(fn_pos_e).' - L.(fn_pos_t).';
    vel_err = L.(fn_vel_e).' - L.(fn_vel_t).';
    if isfield(L, fn_acc_e) && isfield(L, fn_acc_t)
        acc_err = L.(fn_acc_e).' - L.(fn_acc_t).';
    else
        acc_err = gradient(vel_err, L.tt, 1);
    end

    % Statistics -----------------------------------------------------------
    rmse_pos  = sqrt(mean(vecnorm(pos_err,2,2).^2));
    end_pos   = norm(pos_err(end,:));
    rms_res_p = sqrt(mean(pos_err.^2, 'all'));
    max_res_p = max(vecnorm(pos_err,2,2));
    rms_res_v = sqrt(mean(vel_err.^2, 'all'));
    max_res_v = max(vecnorm(vel_err,2,2));

    summary.(frame) = struct('RMSEpos', rmse_pos, 'EndError', end_pos, ...
        'RMSresidPos', rms_res_p, 'MaxresidPos', max_res_p, ...
        'RMSresidVel', rms_res_v, 'MaxresidVel', max_res_v);

    % Print range information matching Python output
    pos_thr = 1.0; vel_thr = 1.0;
    labs = labels{f};
    for j = 1:3
        dp = pos_err(:,j); dv = vel_err(:,j);
        fprintf('%s %s position diff range: %.2f m to %.2f m. ', frame, labs{j}, min(dp), max(dp));
        idx = find(abs(dp) > pos_thr);
        if isempty(idx)
            fprintf('No samples exceed %.1f m\n', pos_thr);
        else
            fprintf('%d samples exceed %.1f m\n', numel(idx), pos_thr);
        end
        fprintf('%s %s velocity diff range: %.2f m/s to %.2f m/s. ', frame, labs{j}, min(dv), max(dv));
        idx = find(abs(dv) > vel_thr);
        if isempty(idx)
            fprintf('No samples exceed %.1f m/s\n', vel_thr);
        else
            fprintf('%d samples exceed %.1f m/s\n', numel(idx), vel_thr);
        end
    end

    % Plot residuals ------------------------------------------------------
    fig = figure('Visible','off','Position',[100 100 1000 750]);
    for j = 1:3
        subplot(3,3,j);   plot(L.tt, pos_err(:,j)); title(labs{j}); ylabel('Position Error [m]'); grid on;
        subplot(3,3,3+j); plot(L.tt, vel_err(:,j)); ylabel('Velocity Error [m/s]'); grid on;
        subplot(3,3,6+j); plot(L.tt, acc_err(:,j)); xlabel('Time [s]'); ylabel('Accel Error [m/s^2]'); grid on;
    end
    sgtitle(sprintf('Task 7 - Fused minus Truth (%s Frame)', frame));
    base = fullfile(results_dir, sprintf('%s_task7_5_diff_truth_fused_over_time_%s', tag, lower(frame)));
    print(fig, [base '.pdf'], '-dpdf', '-bestfit');
    print(fig, [base '.png'], '-dpng');
    close(fig);
end

% Attitude angle plot --------------------------------------------------------
if ~isempty(att_deg)
    fig = figure('Visible','off');
    names = {'Roll','Pitch','Yaw'};
    for k = 1:3
        subplot(3,1,k); plot(L.tt, att_deg(:,k)); ylabel([names{k} ' [deg]']); grid on;
        if k==3, xlabel('Time [s]'); end
    end
    sgtitle('Task 7 - Attitude Angles');
    base = fullfile(results_dir, sprintf('%s_task7_4_attitude_angles_euler', tag));
    print(fig, [base '.pdf'], '-dpdf', '-bestfit');
    print(fig, [base '.png'], '-dpng');
    close(fig);
end

save(fullfile(results_dir, sprintf('%s_task7_summary.mat', tag)), 'summary');

runtime = toc(t_start);
parts = strsplit(tag, '_');
method = parts{end};
rmse_pos = summary.NED.RMSEpos; %#ok<*NASGU>
final_pos = summary.NED.EndError;
rmse_vel = summary.NED.RMSresidVel;
final_vel = summary.NED.MaxresidVel; % use max residual vel as final value for summary
fprintf('[SUMMARY] method=%s rmse_pos=%.3fm final_pos=%.3fm rmse_vel=%.3fm/s final_vel=%.3fm/s runtime=%.2fs\n', ...
    method, rmse_pos, final_pos, rmse_vel, final_vel, runtime);

end
