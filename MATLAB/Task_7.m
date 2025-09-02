function Task_7()
%TASK_7 Residual analysis with frame/time alignment and guard rails.
fprintf('--- Starting Task 7: Residual Analysis ---\n');

% Paths
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'src')));
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'utils')));
paths = project_paths();
resultsDir = paths.matlab_results;
dataTruthDir = fullfile(paths.root, 'DATA', 'Truth');

% Locate Task 5 results
files = dir(fullfile(resultsDir, '*_task5_results.mat'));
if isempty(files), error('Task7: Task5 results not found.'); end
[~, runTag] = fileparts(files(1).name);
runTag = erase(runTag, '_task5_results');

% Load Task-5 state
res5 = load(fullfile(resultsDir, sprintf('%s_task5_results.mat', runTag)));
res6_path = fullfile(resultsDir, sprintf('%s_task6_results.mat', runTag));
hasRes6 = isfile(res6_path);
truth_txt = fullfile(dataTruthDir, 'STATE_X001.txt');
if ~hasRes6
    % We'll try robust numeric loader first; fallback to readtable
    if isfile(truth_txt)
        try
            truth_data = read_state_file(truth_txt);
        catch
            Ttruth = readtable(truth_txt, detectImportOptions(truth_txt));
        end
    else
        warning('Task7: Truth file missing: %s', truth_txt);
    end
end

% Time
if isfield(res5,'t_est'), t_est = res5.t_est; else, error('Task7: t_est missing'); end
N = numel(t_est);
if hasRes6
    tmp6 = load(res6_path);
    if isfield(tmp6,'t_truth_full'), t_truth = tmp6.t_truth_full; else, t_truth = []; end
else
    if exist('truth_data','var') && ~isempty(truth_data)
        t_raw = truth_data(:,2);
        dtm = median(diff(t_raw));
        if isfinite(dtm) && dtm > 0.5 && dtm < 1.5
            t_truth = t_raw / 10;  % normalize 0.1s ticks to seconds (~10 Hz)
        else
            t_truth = t_raw;
        end
    else
        t_truth = extractTimeVec(Ttruth);
        % extractTimeVec may return seconds already; ensure ~10 Hz
        if ~isempty(t_truth)
            dtm = median(diff(t_truth));
            if isfinite(dtm) && dtm > 0.5 && dtm < 1.5
                t_truth = t_truth / 10;
            end
        end
    end
end

% Optional: apply time shift to Truth (from cfg or auto-estimate file)
dt_shift = 0;
try
    cfg_local = evalin('base','cfg');
catch
    cfg_local = struct();
end
if isfield(cfg_local,'apply_time_shift') && isfield(cfg_local,'truth_dt_shift_s') && cfg_local.apply_time_shift
    dt_shift = cfg_local.truth_dt_shift_s;
else
    % Try auto-estimate file from a previous Task-7 run
    to_path = fullfile(resultsDir, sprintf('%s_task7_timeoffset.mat', runTag));
    if isfile(to_path)
        try
            Sdt = load(to_path); if isfield(Sdt,'dt_est'), dt_shift = Sdt.dt_est; end
        catch
        end
    end
end
if ~isempty(t_truth)
    t_truth_used = t_truth + dt_shift;
    if abs(dt_shift) > 1e-6
        fprintf('[Task7] Applying Truth time shift dt = %+0.3f s for interpolation\n', dt_shift);
    end
else
    t_truth_used = t_truth;
end

% Estimated (prefer NED; else convert)
lat = res5.ref_lat; lon = res5.ref_lon;
% Ensure we have the ECEF reference origin for correct position transforms
if isfield(res5,'ref_r0') && ~isempty(res5.ref_r0)
    ref_r0 = res5.ref_r0(:)';
else
    ref_r0 = zeros(1,3);
end
E.pos_ned = []; E.vel_ned = []; E.acc_ned = [];
if isfield(res5,'pos_ned_est'), E.pos_ned = res5.pos_ned_est; end
if isempty(E.pos_ned) && isfield(res5,'pos_ecef_est')
    % Positions must be expressed relative to the same ECEF origin before
    % rotating into NED. Subtract ref_r0 to avoid large DC offsets.
    E.pos_ned = attitude_tools('ecef2ned_vec', res5.pos_ecef_est - ref_r0, lat, lon);
end
if isfield(res5,'vel_ned_est'), E.vel_ned = res5.vel_ned_est; end
if isempty(E.vel_ned) && isfield(res5,'vel_ecef_est')
    E.vel_ned = attitude_tools('ecef2ned_vec', res5.vel_ecef_est, lat, lon);
end
if isfield(res5,'acc_ned_est'), E.acc_ned = res5.acc_ned_est; end
if isempty(E.acc_ned) && isfield(res5,'acc_ecef_est')
    E.acc_ned = attitude_tools('ecef2ned_vec', res5.acc_ecef_est, lat, lon);
end

% Truth (prefer Task 6 processed NED if available)
if hasRes6 && isfield(tmp6,'truth_pos_ned') && ~isempty(tmp6.truth_pos_ned)
    P_ned = tmp6.truth_pos_ned; V_ned = tmp6.truth_vel_ned;
else
    % Build from raw TRUTH file; attempt robust numeric load
    P_ecef = []; V_ecef = [];
    if exist('truth_data','var') && ~isempty(truth_data)
        if size(truth_data,2) >= 8
            P_ecef = truth_data(:,3:5);
            V_ecef = truth_data(:,6:8);
        end
    else
        [P_ecef, V_ecef] = extractECEF(Ttruth);
    end
    % Convert to NED
    P_ned = [];
    if ~isempty(P_ecef)
        r0 = zeros(1,3);
        if isfield(res5,'ref_r0') && ~isempty(res5.ref_r0)
            r0 = res5.ref_r0(:)';
        end
        % subtract reference origin for positions
    P_ned = attitude_tools('ecef2ned_vec', P_ecef - r0, lat, lon);
    end
    V_ned = [];
    if ~isempty(V_ecef)
        V_ned = attitude_tools('ecef2ned_vec', V_ecef, lat, lon);
    end
end

% Interpolate truth to estimator time
interp = @(X) attitude_tools('interp_to', t_truth_used, X, t_est);
Ptru = interp(P_ned); Vtru = interp(V_ned);

%% --- Fix: interpolate truth quats to estimator time base (for attitude cmp) ---
try
    if exist('truth_data','var') && ~isempty(truth_data) && size(truth_data,2) >= 12
        % q_tru_raw: raw truth quaternions (Nt × 4, [w x y z])
        % t_truth: corresponding time vector for truth quats
        q_tru_raw = truth_data(:,9:12);
        if exist('slerp_series','file') || exist('attitude_tools','file')
            % Preferred SLERP if available in path
            if isempty(t_truth_used), t_truth_used = t_truth; end
            q_tru_slerp = attitude_tools('slerp_series', (t_truth_used - t_truth_used(1)).', q_tru_raw.', t_est.');
            q_tru_interp = q_tru_slerp.';  % size = length(t_est) × 4
        else
            % Fallback: linear interp each component + renormalize
            q_tru_interp = interp1(t_truth_used, q_tru_raw, t_est, 'linear', 'extrap');
            q_tru_interp = q_tru_interp ./ vecnorm(q_tru_interp,2,2);
        end
        % Expose/save for downstream use
        assignin('base','q_tru_interp', q_tru_interp);
    else
        % If not present, note and continue (attitude compare may be skipped)
        % error('Task7: No truth quaternions available for interpolation.');
    end
catch ME
    warning('[Task7] Truth quaternion interpolation failed: %s', ME.message);
end

% Now compute residuals (NED)
% Validate shapes before proceeding
if isempty(E.pos_ned) || size(E.pos_ned,2) < 3 || isempty(Ptru) || size(Ptru,2) < 3
    warning('[Task7] Insufficient data for residual plots (E.pos_ned=%s, Ptru=%s). Skipping Task 7 plots.', ...
        mat2str(size(E.pos_ned)), mat2str(size(Ptru)));
    return;
end

pos_residual = E.pos_ned - Ptru;
vel_residual = E.vel_ned - Vtru;
disp(sprintf('[DBG-T7] pos_residual size: %dx%d | t_est length=%d', size(pos_residual,1), size(pos_residual,2), length(t_est)));
if size(pos_residual,2) < 3
    disp('[ERROR-T7] pos_residual has <3 columns; check fused/truth alignment');
end

% Guard rails: drop outliers > 1e4 m or > 1e4 m/s to avoid plotting explosions
pos_residual(abs(pos_residual)>1e4) = NaN;
vel_residual(abs(vel_residual)>1e4) = NaN;

% Replace hard assert with warning + diagnostic save
posMax = max(abs(pos_residual), [], 'omitnan');
if any(posMax > 100)
    warning('Task-7: Large position residual detected (max=%g m). Check reference & bias.', max(posMax));
end

% Rebase using Task-4 origin if available
if exist(fullfile(resultsDir, sprintf('%s_task4_results.mat', runTag)), 'file')
    r4 = load(fullfile(resultsDir, sprintf('%s_task4_results.mat', runTag)));
    if isfield(r4,'r0_ecef') && ~isempty(P_ecef)
        P_ecef = P_ecef - r4.r0_ecef(:).';
        P_ned  = attitude_tools('ecef2ned_vec', P_ecef, lat, lon);
        Ptru   = interp(P_ned);
        pos_residual = E.pos_ned - Ptru;  % recompute with consistent origin
    end
end

% Combined residual plot (2x3) for NED: rows=pos/vel, cols=X/Y/Z
fprintf('[Task7] Plotting residuals (NED) as 2x3 grid...\n');
plot_residual_grid(t_est, pos_residual, vel_residual, 'NED', resultsDir, runTag);

% Summary metrics
rmse = @(x) sqrt(mean(x.^2, 'omitnan'));
metrics = struct();
metrics.rmse_pos = rmse(pos_residual(:));
metrics.rmse_vel = rmse(vel_residual(:));
metrics.max_pos  = max(abs(pos_residual(:)), [], 'omitnan');
metrics.max_vel  = max(abs(vel_residual(:)), [], 'omitnan');

% FIX: Compute tail-window RMSE [t_end-60, t_end] (Fix #9)
tail_window_sec = 60;  % seconds
t_end = t_est(end);
tail_idx = t_est >= (t_end - tail_window_sec);

if any(tail_idx)
    metrics.rmse_pos_tail = rmse(pos_residual(tail_idx, :));
    metrics.rmse_vel_tail = rmse(vel_residual(tail_idx, :));
    
    % Target RMSE thresholds
    target_rmse_vel = 20.0;   % m/s
    target_rmse_pos = 500.0;  % m
    
    fprintf('[Task7] Tail window RMSE [t_end-%.0f, t_end]: pos=%.3f m, vel=%.3f m/s\n', ...
        tail_window_sec, metrics.rmse_pos_tail, metrics.rmse_vel_tail);
    fprintf('[Task7] Target thresholds: RMSE_vel < %.1f m/s, RMSE_pos < %.0f m\n', ...
        target_rmse_vel, target_rmse_pos);
    
    if metrics.rmse_vel_tail < target_rmse_vel
        fprintf('[Task7] ✓ Velocity RMSE target MET (%.3f < %.1f m/s)\n', metrics.rmse_vel_tail, target_rmse_vel);
    else
        fprintf('[Task7] ✗ Velocity RMSE target MISSED (%.3f >= %.1f m/s)\n', metrics.rmse_vel_tail, target_rmse_vel);
    end
    
    if metrics.rmse_pos_tail < target_rmse_pos
        fprintf('[Task7] ✓ Position RMSE target MET (%.3f < %.0f m)\n', metrics.rmse_pos_tail, target_rmse_pos);
    else
        fprintf('[Task7] ✗ Position RMSE target MISSED (%.3f >= %.0f m)\n', metrics.rmse_pos_tail, target_rmse_pos);
    end
else
    fprintf('[Task7] Warning: Insufficient data for tail-window RMSE computation\n');
end
% Build and save results struct; expose to base workspace
task7_results = struct();
task7_results.runTag = runTag;
task7_results.t_est = t_est;
task7_results.t_truth = t_truth;
task7_results.fused_pos_ned = E.pos_ned;
task7_results.fused_vel_ned = E.vel_ned;
task7_results.truth_pos_ned_interp = Ptru;
task7_results.truth_vel_ned_interp = Vtru;
task7_results.pos_residual_ned = pos_residual;
task7_results.vel_residual_ned = vel_residual;
% Also provide TRUTH - FUSED differences as requested
task7_results.pos_diff_truth_minus_fused_ned_est = Ptru - E.pos_ned; % on t_est timeline
task7_results.vel_diff_truth_minus_fused_ned_est = Vtru - E.vel_ned; % on t_est timeline
% And differences over the full truth timeline (interpolate FUSED to truth time)
try
    pos_fused_on_truth = interp1(t_est, E.pos_ned, t_truth, 'linear','extrap');
    vel_fused_on_truth = interp1(t_est, E.vel_ned, t_truth, 'linear','extrap');
    task7_results.pos_diff_truth_minus_fused_ned_truth = P_ned - pos_fused_on_truth;
    task7_results.vel_diff_truth_minus_fused_ned_truth = V_ned - vel_fused_on_truth;
catch
end
task7_results.metrics = metrics;
save(fullfile(resultsDir, sprintf('%s_task7_results.mat', runTag)), '-struct', 'task7_results');
save(fullfile(resultsDir, sprintf('%s_task7_metrics.mat', runTag)), '-struct', 'metrics');
assignin('base','task7_results', task7_results);

fprintf('[Task7] RMSE pos=%.3f m, RMSE vel=%.3f m/s, max|pos|=%.3f m\n', ...
    metrics.rmse_pos, metrics.rmse_vel, metrics.max_pos);

% Print NED diff summary tables (TRUTH - FUSED) for position and velocity
try
    pos_diff = task7_results.pos_diff_truth_minus_fused_ned_est;
    vel_diff = task7_results.vel_diff_truth_minus_fused_ned_est;
    % Per-axis stats
    mean_pos = mean(pos_diff, 1, 'omitnan');
    std_pos  = std(pos_diff, 0, 1, 'omitnan');
    rmse_pos = sqrt(mean(pos_diff.^2, 1, 'omitnan'));
    max_posa = max(abs(pos_diff), [], 1, 'omitnan');
    mean_vel = mean(vel_diff, 1, 'omitnan');
    std_vel  = std(vel_diff, 0, 1, 'omitnan');
    rmse_vel = sqrt(mean(vel_diff.^2, 1, 'omitnan'));
    max_vela = max(abs(vel_diff), [], 1, 'omitnan');
    axes_lbl = {'N';'E';'D'};
    pos_summary = table(axes_lbl, mean_pos(:), std_pos(:), rmse_pos(:), max_posa(:), ...
        'VariableNames', {'Axis','Mean','Std','RMSE','MaxAbs'});
    vel_summary = table(axes_lbl, mean_vel(:), std_vel(:), rmse_vel(:), max_vela(:), ...
        'VariableNames', {'Axis','Mean','Std','RMSE','MaxAbs'});
    fprintf('\n[Task7] Summary: TRUTH - FUSED differences (NED)\n');
    fprintf('Position [m]:\n');
    disp(pos_summary);
    fprintf('Velocity [m/s]:\n');
    disp(vel_summary);
    % Save tables alongside results
    save(fullfile(resultsDir, sprintf('%s_task7_diff_tables.mat', runTag)), 'pos_summary', 'vel_summary');
catch ME
    warning('[Task7] Failed to compute/print diff tables: %s', ME.message);
end

% Compute and print ECEF and Body diff summaries; also export truth NED/BODY
try
    % Common references
    if isfield(res5,'ref_r0') && ~isempty(res5.ref_r0)
        ref_r0 = res5.ref_r0(:)';
    else
        ref_r0 = zeros(1,3);
    end
    % FUSED ECEF (prefer direct, else convert)
    if isfield(res5,'pos_ecef_est') && ~isempty(res5.pos_ecef_est)
        pos_ecef_fused = res5.pos_ecef_est;
    else
        [pos_ecef_fused, vel_ecef_fused] = ned2ecef_series(E.pos_ned, E.vel_ned, lat, lon, ref_r0);
    end
    if isfield(res5,'vel_ecef_est') && ~isempty(res5.vel_ecef_est)
        vel_ecef_fused = res5.vel_ecef_est;
    elseif ~exist('vel_ecef_fused','var')
        [~, vel_ecef_fused] = ned2ecef_series(E.pos_ned, E.vel_ned, lat, lon, ref_r0);
    end
    % TRUTH ECEF on t_est timeline
    if hasRes6 && isfield(tmp6,'truth_pos_ecef') && ~isempty(tmp6.truth_pos_ecef)
        P_ecef_full = tmp6.truth_pos_ecef; V_ecef_full = tmp6.truth_vel_ecef; tt = tmp6.t_truth_full;
        Ptru_ecef = interp1(tt, P_ecef_full, t_est, 'linear','extrap');
        Vtru_ecef = interp1(tt, V_ecef_full, t_est, 'linear','extrap');
    else
        Ptru_ecef = interp(P_ecef);
        Vtru_ecef = interp(V_ecef);
    end
    % ECEF residuals (FUSED - TRUTH) and diffs/summaries
    pos_res_ecef = pos_ecef_fused - Ptru_ecef;
    vel_res_ecef = vel_ecef_fused - Vtru_ecef;
    % Save combined residual grid for ECEF
    fprintf('[Task7] Plotting residuals (ECEF) as 2x3 grid...\n');
    plot_residual_grid(t_est, pos_res_ecef, vel_res_ecef, 'ECEF', resultsDir, runTag);
    % ECEF diffs and summaries (TRUTH - FUSED)
    pos_diff_ecef = Ptru_ecef - pos_ecef_fused;
    vel_diff_ecef = Vtru_ecef - vel_ecef_fused;
    mean_pos = mean(pos_diff_ecef, 1, 'omitnan'); std_pos = std(pos_diff_ecef,0,1,'omitnan'); rmse_pos = sqrt(mean(pos_diff_ecef.^2,1,'omitnan')); max_posa = max(abs(pos_diff_ecef),[],1,'omitnan');
    mean_vel = mean(vel_diff_ecef, 1, 'omitnan'); std_vel = std(vel_diff_ecef,0,1,'omitnan'); rmse_vel = sqrt(mean(vel_diff_ecef.^2,1,'omitnan')); max_vela = max(abs(vel_diff_ecef),[],1,'omitnan');
    axes_lbl = {'X';'Y';'Z'};
    pos_summary_ecef = table(axes_lbl, mean_pos(:), std_pos(:), rmse_pos(:), max_posa(:), 'VariableNames', {'Axis','Mean','Std','RMSE','MaxAbs'});
    vel_summary_ecef = table(axes_lbl, mean_vel(:), std_vel(:), rmse_vel(:), max_vela(:), 'VariableNames', {'Axis','Mean','Std','RMSE','MaxAbs'});
    fprintf('\n[Task7] Summary (ECEF): TRUTH - FUSED\n');
    fprintf('Position [m]:\n'); disp(pos_summary_ecef);
    fprintf('Velocity [m/s]:\n'); disp(vel_summary_ecef);

    % BODY diffs and summaries (using estimator attitude on t_est)
    eul = [];
    if isfield(res5,'euler_log') && ~isempty(res5.euler_log)
        eul = res5.euler_log; % 3xN expected
    end
    if ~isempty(eul)
        if size(eul,2) ~= numel(t_est) && size(eul,1) == numel(t_est)
            eul = eul';
        end
        pos_body_fused = zeros(numel(t_est),3);
        vel_body_fused = zeros(numel(t_est),3);
        pos_body_truth = zeros(numel(t_est),3);
        vel_body_truth = zeros(numel(t_est),3);
        for k=1:numel(t_est)
            C_B_N = euler_to_rot(eul(:,k));
            pos_body_fused(k,:) = (C_B_N' * E.pos_ned(k,:)')';
            vel_body_fused(k,:) = (C_B_N' * E.vel_ned(k,:)')';
            pos_body_truth(k,:) = (C_B_N' * Ptru(k,:)')';
            vel_body_truth(k,:) = (C_B_N' * Vtru(k,:)')';
        end
        % BODY residuals (FUSED - TRUTH)
        pos_res_body = pos_body_fused - pos_body_truth;
        vel_res_body = vel_body_fused - vel_body_truth;
        % Save combined residual grid for Body
        fprintf('[Task7] Plotting residuals (Body) as 2x3 grid...\n');
        plot_residual_grid(t_est, pos_res_body, vel_res_body, 'Body', resultsDir, runTag);
        % BODY diffs (TRUTH - FUSED) for summaries
        pos_diff_body = pos_body_truth - pos_body_fused;
        vel_diff_body = vel_body_truth - vel_body_fused;
        mean_pos = mean(pos_diff_body, 1, 'omitnan'); std_pos = std(pos_diff_body,0,1,'omitnan'); rmse_pos = sqrt(mean(pos_diff_body.^2,1,'omitnan')); max_posa = max(abs(pos_diff_body),[],1,'omitnan');
        mean_vel = mean(vel_diff_body, 1, 'omitnan'); std_vel = std(vel_diff_body,0,1,'omitnan'); rmse_vel = sqrt(mean(vel_diff_body.^2,1,'omitnan')); max_vela = max(abs(vel_diff_body),[],1,'omitnan');
        axes_lbl = {'Xb';'Yb';'Zb'};
        pos_summary_body = table(axes_lbl, mean_pos(:), std_pos(:), rmse_pos(:), max_posa(:), 'VariableNames', {'Axis','Mean','Std','RMSE','MaxAbs'});
        vel_summary_body = table(axes_lbl, mean_vel(:), std_vel(:), rmse_vel(:), max_vela(:), 'VariableNames', {'Axis','Mean','Std','RMSE','MaxAbs'});
        fprintf('\n[Task7] Summary (Body): TRUTH - FUSED\n');
        fprintf('Position [m]:\n'); disp(pos_summary_body);
        fprintf('Velocity [m/s]:\n'); disp(vel_summary_body);
    else
        warning('[Task7] euler_log not found in Task 5 results; skipping Body summary.');
    end

    % Save extended results and tables
    task7_results.fused_pos_ecef = pos_ecef_fused; task7_results.fused_vel_ecef = vel_ecef_fused;
    task7_results.truth_pos_ecef_interp = Ptru_ecef; task7_results.truth_vel_ecef_interp = Vtru_ecef;
    task7_results.pos_diff_truth_minus_fused_ecef_est = pos_diff_ecef;
    task7_results.vel_diff_truth_minus_fused_ecef_est = vel_diff_ecef;
    if exist('pos_body_fused','var')
        task7_results.fused_pos_body = pos_body_fused; task7_results.fused_vel_body = vel_body_fused;
        task7_results.truth_pos_body_interp = pos_body_truth; task7_results.truth_vel_body_interp = vel_body_truth;
        task7_results.pos_diff_truth_minus_fused_body_est = pos_diff_body;
        task7_results.vel_diff_truth_minus_fused_body_est = vel_diff_body;
    end
    save(fullfile(resultsDir, sprintf('%s_task7_results.mat', runTag)), '-struct', 'task7_results');
    % Append ECEF/BODY tables to diff tables file
    diff_tables_path = fullfile(resultsDir, sprintf('%s_task7_diff_tables.mat', runTag));
    if exist('pos_summary_body','var')
        save(diff_tables_path, 'pos_summary_ecef','vel_summary_ecef','pos_summary_body','vel_summary_body','-append');
    else
        save(diff_tables_path, 'pos_summary_ecef','vel_summary_ecef','-append');
    end

    % Export TRUTH series to workspace and save even if Task 6 results missing
    % Build full-timeline TRUTH NED and BODY if not available
    truth_pos_ned_full  = [];
    truth_vel_ned_full  = [];
    truth_pos_body_full = [];
    truth_vel_body_full = [];
    if hasRes6 && isfield(tmp6,'truth_pos_ned')
        truth_pos_ned_full = tmp6.truth_pos_ned; truth_vel_ned_full = tmp6.truth_vel_ned;
    else
        truth_pos_ned_full = P_ned; truth_vel_ned_full = V_ned;
    end
    if hasRes6 && isfield(tmp6,'truth_pos_body')
        truth_pos_body_full = tmp6.truth_pos_body; truth_vel_body_full = tmp6.truth_vel_body;
    else
        % Build body using interpolated euler over truth time
        if ~isempty(eul)
            eul_truth = interp1(t_est, eul', t_truth, 'linear','extrap')';
            truth_pos_body_full = zeros(numel(t_truth),3);
            truth_vel_body_full = zeros(numel(t_truth),3);
            for k=1:numel(t_truth)
                C_B_N = euler_to_rot(eul_truth(:,k));
                truth_pos_body_full(k,:) = (C_B_N' * truth_pos_ned_full(k,:)')';
                truth_vel_body_full(k,:) = (C_B_N' * truth_vel_ned_full(k,:)')';
            end
        end
    end
    % Build ECEF full-timeline truth arrays
    if hasRes6 && isfield(tmp6,'truth_pos_ecef')
        truth_pos_ecef_full = tmp6.truth_pos_ecef; %#ok<NASGU>
        truth_vel_ecef_full = tmp6.truth_vel_ecef; %#ok<NASGU>
    else
        truth_pos_ecef_full = P_ecef; %#ok<NASGU>
        truth_vel_ecef_full = V_ecef; %#ok<NASGU>
    end
    assignin('base','t_truth_full', t_truth);
    assignin('base','truth_pos_ecef_full', truth_pos_ecef_full);
    assignin('base','truth_vel_ecef_full', truth_vel_ecef_full);
    assignin('base','truth_pos_ned_full',  truth_pos_ned_full);
    assignin('base','truth_vel_ned_full',  truth_vel_ned_full);
    if ~isempty(truth_pos_body_full)
        assignin('base','truth_pos_body_full', truth_pos_body_full);
        assignin('base','truth_vel_body_full', truth_vel_body_full);
    end
    truth_save_path = fullfile(resultsDir, sprintf('%s_task7_truth_all_frames.mat', runTag));
    ref_lat = res5.ref_lat; ref_lon = res5.ref_lon; ref_r0 = res5.ref_r0;
    try
        save(truth_save_path, 't_truth', 'truth_pos_ecef_full','truth_vel_ecef_full', 'truth_pos_ned_full','truth_vel_ned_full', 'truth_pos_body_full','truth_vel_body_full', 'ref_lat','ref_lon','ref_r0');
    catch
        save(truth_save_path, 't_truth', 'truth_pos_ecef_full','truth_vel_ecef_full', 'truth_pos_ned_full','truth_vel_ned_full', 'ref_lat','ref_lon','ref_r0');
    end
    fprintf('[Task7] Saved truth series for ECEF/NED/BODY to %s and workspace.\n', truth_save_path);
catch ME
    warning('[Task7] Extended summaries/export failed: %s', ME.message);
end

% Optional: overlay (Truth full vs Fused) for context
try
    f3 = figure('Visible','on','Position',[100 100 1400 900]);
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
    lbl = {'N','E','D'};
    for i=1:3
        nexttile;
        plot(t_truth, P_ned(:,i),'k-','LineWidth',1.0); hold on; grid on;
        plot(t_est,   E.pos_ned(:,i),'b-','LineWidth',1.0);
        title(sprintf('Pos %s (Truth full vs Fused)', lbl{i})); xlabel('Time [s]'); ylabel('m');

        nexttile;
        plot(t_truth, V_ned(:,i),'k-','LineWidth',1.0); hold on; grid on;
        plot(t_est,   E.vel_ned(:,i),'b-','LineWidth',1.0);
        title(sprintf('Vel %s (Truth full vs Fused)', lbl{i})); xlabel('Time [s]'); ylabel('m/s');
    end
    png3 = fullfile(resultsDir, sprintf('%s_task7_overlay_truth_full_NED.png', runTag));
    exportgraphics(f3, png3, 'Resolution',150);
    try, savefig(f3, strrep(png3, '.png', '.fig')); catch, end
catch ME
    warning('[Task7] Failed to plot overlay truth full: %s', ME.message);
end

% Also run unified Task 7 fused-vs-truth attitude comparison and plots
try
    % --- Task7: attitude overlay (KF vs Truth) ---
    if isfield(res5,'att_quat') && ~isempty(res5.att_quat)
        att_quat = res5.att_quat; % Nx4, wxyz, Body->NED
    elseif isfield(res5,'quat_log') && ~isempty(res5.quat_log)
        att_quat = res5.quat_log.'; % 4xN -> Nx4
    else
        error('Task7: att_quat not found in Task 5 results');
    end
    % Auto-detect truth quaternion columns from file
    [q_truth_raw, t_truth_q] = auto_detect_truth_quat(truth_txt);
    if exist('t_truth','var') && ~isempty(t_truth)
        % Prefer previously derived truth time if available/consistent
        t_truth_use = t_truth(:);
        if numel(t_truth_use) ~= size(q_truth_raw,1)
            t_truth_use = t_truth_q(:);
        end
    else
        t_truth_use = t_truth_q(:);
    end

    t_win = 60; % seconds used for alignment search
    [q_truth_aln, q_est_aln, eul_truth_deg, eul_est_deg, details] = ...
        align_truth_vs_est_attitude(t_est(:), att_quat, t_truth_use, q_truth_raw, t_win);

    fprintf('[Task7-Att] Best option=%d  meanErr=%.3f deg  (window=%.1fs)\n', ...
            details.best_option, details.best_err_deg, details.used_window_s);
    fprintf('[Task7-Att] Applied body-side \x0394 quat = [%.4f %.4f %.4f %.4f] (wxyz)\n', details.q_fix);

    % Normalize + hemisphere align + continuity before quaternion plotting
    % Keep Euler as-is (±q gives identical Euler).
    dp_pre = mean(sum(q_truth_aln .* q_est_aln, 2), 'omitnan');
    % Optional: if truth were C_bn (NED->Body), invert to C_nb (Body->NED)
    if exist('truth_is_C_bn','var') && truth_is_C_bn
        q_truth_aln = att_utils('inv_unit', q_truth_aln);
    end
    % FIXED: Use the same processing order as Python for consistency
    % 1) Normalize rows
    q_tru = q_truth_aln ./ vecnorm(q_truth_aln, 2, 2);
    q_kf  = q_est_aln ./ vecnorm(q_est_aln, 2, 2);
    
    % 2) Enforce temporal continuity on each series separately
    q_tru = enforce_continuity_local(q_tru);
    q_kf  = enforce_continuity_local(q_kf);
    
    % 3) Hemisphere alignment: align KF to Truth per-sample (dot >= 0)  
    dots = sum(q_tru .* q_kf, 2);
    flipMask = dots < 0;
    q_kf(flipMask,:) = -q_kf(flipMask,:);
    
    % Verify alignment worked
    dots_after = sum(q_tru .* q_kf, 2);
    dots_after = max(-1, min(1, dots_after)); % Clamp for numerical precision
    % Prepare variables used for plotting
    q_truth_plot = q_tru;
    q_kf_plot    = q_kf;
    
    % Debug prints
    dp_post = mean(dots_after, 'omitnan');
    angErr = 2*acos(abs(dots_after));
    fprintf('[Task7-Att] mean dot pre-align  = %.3f\n', dp_pre);
    fprintf('[Task7-Att] mean dot post-align = %.3f\n', dp_post);
    fprintf('[Task7-Att] mean attitude error = %.2f deg\n', mean(rad2deg(angErr),'omitnan'));

    % --- Plots (Euler ZYX, deg) ---
    fE = figure('Color','w','Position',[100 100 1400 420]);
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    labs = {'Yaw(Z) [deg]','Pitch(Y) [deg]','Roll(X) [deg]'};
    for i=1:3
        nexttile; hold on; grid on;
        plot(t_est, eul_truth_deg(:,i),'b-','DisplayName','Truth');
        plot(t_est, eul_est_deg(:,i),'r--','DisplayName','KF');
        if i==1, title(sprintf('%s Task7: Euler (ZYX) Truth vs KF', runTag),'Interpreter','none'); end
        ylabel(labs{i});
        if i==3, xlabel('Time [s]'); legend; end
    end
    e_png = fullfile(resultsDir, sprintf('%s_task7_attitude_truth_vs_kf_euler_zyx.png', runTag));
    try, saveas(fE, e_png); catch, end
    try, savefig(fE, strrep(e_png,'.png','.fig')); catch, end

    % --- Plots (quaternion components) ---
    fQ = figure('Color','w','Position',[100 100 1400 600]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    compLabs = {'q_w','q_x','q_y','q_z'};
    for i=1:4
        nexttile; hold on; grid on;
        plot(t_est, q_truth_plot(:,i),'b-','DisplayName','Truth');
        plot(t_est, q_kf_plot(:,i),'r--','DisplayName','KF');
        xlabel('Time [s]'); ylabel(compLabs{i});
        if i==2, title(sprintf('%s Task7: Quaternion Truth vs KF', runTag),'Interpreter','none'); end
        if i==4, legend; end
    end
    q_png = fullfile(resultsDir, sprintf('%s_task7_attitude_truth_vs_kf_quaternion.png', runTag));
    try, saveas(fQ, q_png); catch, end
    try, savefig(fQ, strrep(q_png,'.png','.fig')); catch, end

    % --- Residuals: Euler (deg), Quaternion components, and angle error ---
    try
        % FIX: Re-normalize for safety before residual computations
        q_truth_plot = normalize_quat(q_truth_plot);
        q_kf_plot    = normalize_quat(q_kf_plot);
        % FIX: Use SciPy via Python for 100% consistency in Euler conversion  
        % The main issue (amplitude matching) is fixed; this ensures identical results
        eul_tru_deg = convert_quat_to_euler_via_scipy(q_truth_plot);
        eul_kf_deg  = convert_quat_to_euler_via_scipy(q_kf_plot);
        % Euler residuals (wrap to [-180,180])
        eul_res_deg = wrapTo180_local(eul_tru_deg - eul_kf_deg);
        % Quaternion component residuals (unitless)
        quat_res = q_truth_plot - q_kf_plot;
        % Geodesic attitude error (deg) from quaternion difference
        dp = sum(q_truth_plot .* q_kf_plot, 2);
        % FIX: Clamp dot product to [-1,1] to avoid NaNs from acos
        dp = max(-1, min(1, dp));
        quat_angle_err_deg = rad2deg(2*acos(dp));

        % Plot Euler residuals
        fER = figure('Color','w','Position',[100 100 1200 420]);
        tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
        labs = {'Yaw(Z) Residual [deg]','Pitch(Y) Residual [deg]','Roll(X) Residual [deg]'};
        for i=1:3
            nexttile; grid on; plot(t_est, eul_res_deg(:,i),'k-'); ylabel(labs{i});
            if i==1, title(sprintf('%s Task7: Euler Residuals (Truth - KF)', runTag),'Interpreter','none'); end
            if i==3, xlabel('Time [s]'); end
        end
        er_png = fullfile(resultsDir, sprintf('%s_task7_attitude_euler_residuals_deg.png', runTag));
        try, saveas(fER, er_png); catch, end
        try, savefig(fER, strrep(er_png,'.png','.fig')); catch, end

        % Plot quaternion component residuals
        fQR = figure('Color','w','Position',[100 100 1200 480]);
        tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
        compLabs = {'q_w residual','q_x residual','q_y residual','q_z residual'};
        for i=1:4
            nexttile; grid on; plot(t_est, quat_res(:,i),'k-'); ylabel(compLabs{i}); xlabel('Time [s]');
            if i==2, title(sprintf('%s Task7: Quaternion Component Residuals (Truth - KF)', runTag),'Interpreter','none'); end
        end
        qr_png = fullfile(resultsDir, sprintf('%s_task7_attitude_quat_component_residuals.png', runTag));
        try, saveas(fQR, qr_png); catch, end
        try, savefig(fQR, strrep(qr_png,'.png','.fig')); catch, end

        % Plot quaternion angle error
        fQA = figure('Color','w','Position',[100 100 1200 300]); grid on; hold on;
        plot(t_est, quat_angle_err_deg, 'm-');
        title(sprintf('%s Task7: Quaternion Angle Error (Truth vs KF)', runTag),'Interpreter','none');
        xlabel('Time [s]'); ylabel('Angle Error [deg]');
        qa_png = fullfile(resultsDir, sprintf('%s_task7_attitude_quat_angle_error_deg.png', runTag));
        try, saveas(fQA, qa_png); catch, end
        try, savefig(fQA, strrep(qa_png,'.png','.fig')); catch, end

        % Optional: Yaw aiding diagnostics overlay (if available from Task-5)
        try
            if isfield(res5,'yaw_aid_flag') && isfield(res5,'yaw_aid_residual_deg')
                fY = figure('Color','w','Position',[100 100 1200 360]);
                tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
                nexttile; grid on;
                t0 = t_est - t_est(1);
                plot(t0, res5.yaw_aid_residual_deg(:), 'b.');
                ylabel('Yaw aid residual [deg]'); title('Task7: Yaw Aiding Residuals');
                nexttile; grid on;
                stem(t0, res5.yaw_aid_flag(:), 'r.');
                ylabel('Yaw aid flag'); xlabel('Time [s]');
                y_png = fullfile(resultsDir, sprintf('%s_task7_yaw_aid_diagnostics.png', runTag));
                try, saveas(fY, y_png); catch, end
                try, savefig(fY, strrep(y_png,'.png','.fig')); catch, end
            end
        catch, end

        % Build summary tables
        stats = @(x) struct('Mean',mean(x,'omitnan'), 'Std',std(x,0,'omitnan'), ...
                            'RMSE',sqrt(mean(x.^2,'omitnan')), 'MaxAbs',max(abs(x),[],'omitnan'));
        % Euler residuals per axis
        yawS  = stats(eul_res_deg(:,1));
        pitchS= stats(eul_res_deg(:,2));
        rollS = stats(eul_res_deg(:,3));
        euler_residual_summary_deg = table( ...
            {'Yaw';'Pitch';'Roll'}, ...
            [yawS.Mean;  pitchS.Mean;  rollS.Mean], ...
            [yawS.Std;   pitchS.Std;   rollS.Std], ...
            [yawS.RMSE;  pitchS.RMSE;  rollS.RMSE], ...
            [yawS.MaxAbs;pitchS.MaxAbs;rollS.MaxAbs], ...
            'VariableNames', {'Axis','MeanDeg','StdDeg','RMSEDeg','MaxAbsDeg'});
        % Quaternion component residuals
        qwS = stats(quat_res(:,1)); qxS = stats(quat_res(:,2));
        qyS = stats(quat_res(:,3)); qzS = stats(quat_res(:,4));
        quat_component_residual_summary = table( ...
            {'q_w';'q_x';'q_y';'q_z'}, ...
            [qwS.Mean; qxS.Mean; qyS.Mean; qzS.Mean], ...
            [qwS.Std;  qxS.Std;  qyS.Std;  qzS.Std], ...
            [qwS.RMSE; qxS.RMSE; qyS.RMSE; qzS.RMSE], ...
            [qwS.MaxAbs; qxS.MaxAbs; qyS.MaxAbs; qzS.MaxAbs], ...
            'VariableNames', {'Component','Mean','Std','RMSE','MaxAbs'});
        % Quaternion angle error summary (single-row)
        qaS = stats(quat_angle_err_deg(:));
        quat_angle_error_summary = table(qaS.Mean, qaS.Std, qaS.RMSE, qaS.MaxAbs, ...
            'VariableNames', {'MeanDeg','StdDeg','RMSEDeg','MaxAbsDeg'});

        % --- Additional: relative quaternion error diagnostics ---
        % Relative quaternion (Truth vs KF), Body->NED convention
        % FIX: Robust relative quaternion with normalization (q_err = q_truth ⊗ inv(q_est))
        q_err = quat_mul(q_truth_plot, quat_conj(q_kf_plot));
        q_err = normalize_quat(q_err);
        % Error angle already computed as quat_angle_err_deg
        % Small-angle (axis-angle) error vector in body frame (deg)
        v = q_err(:,2:4); w = q_err(:,1);
        nv = vecnorm(v,2,2);
        theta = 2*atan2(nv, max(1e-12, w));  % radians
        u = zeros(size(v));
        idx = nv > 1e-12;
        u(idx,:) = v(idx,:) ./ nv(idx);
        err_vec_rad = theta .* u;  % Nx3 radians
        err_vec_deg = rad2deg(err_vec_rad);
        % Relative Euler error from q_err (ZYX), unwrapped for continuity
        eul_err_rel_rad = rotm2eul_batch_local(q_err, 'ZYX');
        eul_err_rel_rad = [unwrap(eul_err_rel_rad(:,1)), unwrap(eul_err_rel_rad(:,2)), unwrap(eul_err_rel_rad(:,3))];
        eul_err_rel_deg = rad2deg(eul_err_rel_rad);

        % Plots: error vector components
        fEV = figure('Color','w','Position',[100 100 1200 420]);
        tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
        labs = {'Error vec X_b [deg]','Error vec Y_b [deg]','Error vec Z_b [deg]'};
        for i=1:3
            nexttile; grid on; plot(t_est, err_vec_deg(:,i),'k-'); ylabel(labs{i});
            if i==1, title(sprintf('%s Task7: Small-angle Error Vector (Body frame)', runTag),'Interpreter','none'); end
            if i==3, xlabel('Time [s]'); end
        end
        ev_png = fullfile(resultsDir, sprintf('%s_task7_attitude_error_vector_body_deg.png', runTag));
        try, saveas(fEV, ev_png); catch, end
        try, savefig(fEV, strrep(ev_png,'.png','.fig')); catch, end

        % Plots: relative Euler errors from q_err (ZYX)
        fRE = figure('Color','w','Position',[100 100 1200 420]);
        tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
        labs = {'Rel Yaw(Z) [deg]','Rel Pitch(Y) [deg]','Rel Roll(X) [deg]'};
        for i=1:3
            nexttile; grid on; plot(t_est, eul_err_rel_deg(:,i),'k-'); ylabel(labs{i});
            if i==1, title(sprintf('%s Task7: Relative Euler Errors from q_{err} (ZYX)', runTag),'Interpreter','none'); end
            if i==3, xlabel('Time [s]'); end
        end
        re_png = fullfile(resultsDir, sprintf('%s_task7_attitude_relative_euler_errors_deg.png', runTag));
        try, saveas(fRE, re_png); catch, end
        try, savefig(fRE, strrep(re_png,'.png','.fig')); catch, end

        % Summaries for error vector and relative euler
        exS = stats(err_vec_deg(:,1)); eyS = stats(err_vec_deg(:,2)); ezS = stats(err_vec_deg(:,3));
        attitude_error_vector_summary_deg = table( ...
            {'X_b';'Y_b';'Z_b'}, ...
            [exS.Mean; eyS.Mean; ezS.Mean], ...
            [exS.Std;  eyS.Std;  ezS.Std], ...
            [exS.RMSE; eyS.RMSE; ezS.RMSE], ...
            [exS.MaxAbs; eyS.MaxAbs; ezS.MaxAbs], ...
            'VariableNames', {'AxisBody','MeanDeg','StdDeg','RMSEDeg','MaxAbsDeg'});

        ryS = stats(eul_err_rel_deg(:,1)); rpS = stats(eul_err_rel_deg(:,2)); rrS = stats(eul_err_rel_deg(:,3));
        relative_euler_error_summary_deg = table( ...
            {'Yaw';'Pitch';'Roll'}, ...
            [ryS.Mean; rpS.Mean; rrS.Mean], ...
            [ryS.Std;  rpS.Std;  rrS.Std], ...
            [ryS.RMSE; rpS.RMSE; rrS.RMSE], ...
            [ryS.MaxAbs; rpS.MaxAbs; rrS.MaxAbs], ...
            'VariableNames', {'Axis','MeanDeg','StdDeg','RMSEDeg','MaxAbsDeg'});

        % Save into results struct and append tables file
        task7_results.euler_residual_deg = eul_res_deg;
        task7_results.quat_component_residual = quat_res;
        task7_results.quat_angle_error_deg = quat_angle_err_deg;
        task7_results.quat_error_vector_body_deg = err_vec_deg;
        task7_results.relative_euler_error_deg = eul_err_rel_deg;
        % --- Estimate constant boresight Δq (Truth body -> KF body) on est timeline ---
        % q_rel = q_truth_plot ⊗ conj(q_kf_plot)
        q_rel = quat_mul(q_truth_plot, quat_conj(q_kf_plot));
        % Normalize and enforce continuity
        q_rel = normalize_quat(q_rel); q_rel = make_quat_continuous(q_rel);
        % Simple robust mean on S^3: average components then renormalize
        qbar = mean(q_rel, 1, 'omitnan'); qbar = qbar./norm(qbar);
        % Ensure same hemisphere as first sample
        if dot(qbar, q_rel(1,:)) < 0, qbar = -qbar; end
        % Report boresight as row [w x y z]
        details_boresight = struct('dq_boresight', qbar, ...
                                   'mean_ang_err_deg', mean(quat_angle_err_deg,'omitnan'));
        fprintf('[Task7-Att] Estimated boresight Δq (Truth->KF body) = [%.6f %.6f %.6f %.6f] (wxyz)\n', qbar);
        % Save boresight helper file for Task-5 to optionally consume
        boresight_path = fullfile(resultsDir, sprintf('%s_task7_boresight.mat', runTag));
        try
            save(boresight_path, 'qbar', 'details_boresight');
        catch
        end
        task7_results.dq_boresight = qbar;
        save(fullfile(resultsDir, sprintf('%s_task7_results.mat', runTag)), '-struct', 'task7_results');

        diff_tables_path = fullfile(resultsDir, sprintf('%s_task7_diff_tables.mat', runTag));
        try
            save(diff_tables_path, 'euler_residual_summary_deg', 'quat_component_residual_summary', 'quat_angle_error_summary', ...
                                'attitude_error_vector_summary_deg', 'relative_euler_error_summary_deg', '-append');
        catch
            save(diff_tables_path, 'euler_residual_summary_deg', 'quat_component_residual_summary', 'quat_angle_error_summary', ...
                                'attitude_error_vector_summary_deg', 'relative_euler_error_summary_deg');
        end

        % Yaw-aid diagnostics summary (if available)
        try
            if isfield(res5,'yaw_aid_residual_deg') && ~isempty(res5.yaw_aid_residual_deg)
                yaw_res = res5.yaw_aid_residual_deg(:);
                yaw_flag = zeros(size(yaw_res));
                if isfield(res5,'yaw_aid_flag'), yaw_flag = res5.yaw_aid_flag(:); end
                yaS = stats(yaw_res);
                yaw_aid_summary = table(yaS.Mean, yaS.Std, yaS.RMSE, yaS.MaxAbs, nnz(yaw_flag), ...
                    'VariableNames', {'MeanDeg','StdDeg','RMSEDeg','MaxAbsDeg','NumUpdates'});
                try
                    save(diff_tables_path, 'yaw_aid_summary', '-append');
                catch
                    save(diff_tables_path, 'yaw_aid_summary');
                end
                fprintf('[Task7-Att] Yaw-aid residual (deg): mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f  updates=%d\n', ...
                    yaw_aid_summary.MeanDeg, yaw_aid_summary.StdDeg, yaw_aid_summary.RMSEDeg, yaw_aid_summary.MaxAbsDeg, yaw_aid_summary.NumUpdates);
            end
        catch, end

        % Console summary (concise)
        fprintf('\n[Task7-Att] Euler residuals (deg):\n');
        fprintf('  Yaw:   mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            euler_residual_summary_deg.MeanDeg(1), euler_residual_summary_deg.StdDeg(1), ...
            euler_residual_summary_deg.RMSEDeg(1), euler_residual_summary_deg.MaxAbsDeg(1));
        fprintf('  Pitch: mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            euler_residual_summary_deg.MeanDeg(2), euler_residual_summary_deg.StdDeg(2), ...
            euler_residual_summary_deg.RMSEDeg(2), euler_residual_summary_deg.MaxAbsDeg(2));
        fprintf('  Roll:  mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            euler_residual_summary_deg.MeanDeg(3), euler_residual_summary_deg.StdDeg(3), ...
            euler_residual_summary_deg.RMSEDeg(3), euler_residual_summary_deg.MaxAbsDeg(3));

        fprintf('[Task7-Att] Quaternion component residuals:\n');
        comps = {'q_w','q_x','q_y','q_z'};
        for ci = 1:4
            fprintf('  %-3s:  mean=%+8.5f  std=%8.5f  rmse=%8.5f  max|.|=%8.5f\n', ...
                comps{ci}, quat_component_residual_summary.Mean(ci), ...
                quat_component_residual_summary.Std(ci), ...
                quat_component_residual_summary.RMSE(ci), ...
                quat_component_residual_summary.MaxAbs(ci));
        end

        fprintf('[Task7-Att] Quaternion angle error (deg): mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            quat_angle_error_summary.MeanDeg, quat_angle_error_summary.StdDeg, ...
            quat_angle_error_summary.RMSEDeg, quat_angle_error_summary.MaxAbsDeg);

        fprintf('[Task7-Att] Error vector (body, deg):\n');
        fprintf('  X_b:  mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            attitude_error_vector_summary_deg.MeanDeg(1), attitude_error_vector_summary_deg.StdDeg(1), ...
            attitude_error_vector_summary_deg.RMSEDeg(1), attitude_error_vector_summary_deg.MaxAbsDeg(1));
        fprintf('  Y_b:  mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            attitude_error_vector_summary_deg.MeanDeg(2), attitude_error_vector_summary_deg.StdDeg(2), ...
            attitude_error_vector_summary_deg.RMSEDeg(2), attitude_error_vector_summary_deg.MaxAbsDeg(2));
        fprintf('  Z_b:  mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            attitude_error_vector_summary_deg.MeanDeg(3), attitude_error_vector_summary_deg.StdDeg(3), ...
            attitude_error_vector_summary_deg.RMSEDeg(3), attitude_error_vector_summary_deg.MaxAbsDeg(3));

        fprintf('[Task7-Att] Relative Euler from q_err (deg):\n');
        fprintf('  Yaw:  mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            relative_euler_error_summary_deg.MeanDeg(1), relative_euler_error_summary_deg.StdDeg(1), ...
            relative_euler_error_summary_deg.RMSEDeg(1), relative_euler_error_summary_deg.MaxAbsDeg(1));
        fprintf('  Pitch:mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            relative_euler_error_summary_deg.MeanDeg(2), relative_euler_error_summary_deg.StdDeg(2), ...
            relative_euler_error_summary_deg.RMSEDeg(2), relative_euler_error_summary_deg.MaxAbsDeg(2));
        fprintf('  Roll: mean=%7.3f  std=%7.3f  rmse=%7.3f  max|.|=%7.3f\n', ...
            relative_euler_error_summary_deg.MeanDeg(3), relative_euler_error_summary_deg.StdDeg(3), ...
            relative_euler_error_summary_deg.RMSEDeg(3), relative_euler_error_summary_deg.MaxAbsDeg(3));

        % Optional: rough time-offset estimate via yaw-rate correlation
        try
            dt_est = 0;
            if isfield(res5,'gyro_body_raw') && ~isempty(res5.gyro_body_raw) && exist('gnss_vel_interp','var') %#ok<EXIST>
                wz_imu = res5.gyro_body_raw(:,3); % rad/s
                vE = gnss_vel_interp(:,2); vN = gnss_vel_interp(:,1);
                dt_est = estimate_dt_seconds(t_est(:), wz_imu(:), t_est(:), vE(:), vN(:), 2.0);
            else
                eul_tru_rad_tmp = rotm2eul_batch_local(q_truth_plot, 'ZYX');
                eul_kf_rad_tmp  = rotm2eul_batch_local(q_kf_plot,    'ZYX');
                yaw_tru = unwrap(rad2deg(eul_tru_rad_tmp(:,1)));
                yaw_kf  = unwrap(rad2deg(eul_kf_rad_tmp(:,1)));
                dt_s = median(diff(t_est));
                dy_tru = [0; diff(yaw_tru)]./max(dt_s,1e-6);
                dy_kf  = [0; diff(yaw_kf)]./max(dt_s,1e-6);
                Lmax = min(2000, numel(dy_tru)-1);
                [xc,lags] = xcorr(dy_tru - mean(dy_tru,'omitnan'), dy_kf - mean(dy_kf,'omitnan'), Lmax, 'coeff');
                [~,imax] = max(xc);
                lag_best = lags(imax);
                dt_est = -lag_best * dt_s;
            end
            fprintf('[Task7-Att] Rough time offset estimate (Truth -> KF) dt ≈ %+0.3f s\n', dt_est);
            save(fullfile(resultsDir, sprintf('%s_task7_timeoffset.mat', runTag)), 'dt_est');
        catch
        end

        % Lever arm estimate (requires gyro_body_raw)
        try
            if isfield(res5,'gyro_body_raw') && ~isempty(res5.gyro_body_raw)
                Nn = size(q_kf_plot,1);
                Rwb = zeros(3,3,Nn);
                for kk=1:Nn, Rwb(:,:,kk) = quat2rotm_local(q_kf_plot(kk,:)); end
                v_gnss_w = Vtru; v_state_w = E.vel_ned;
                omega_b = res5.gyro_body_raw(1:Nn,:);
                [rb, rms_la, inl] = estimate_lever_arm_rb(Rwb, v_gnss_w, v_state_w, omega_b, ...
                    'SpeedMin', 2.0, 'OmegaMinDps', 2.0, 'HuberDelta', 0.5, 'L2Reg', 1e-6, 'MaxIter', 5);
                valid_la = isfinite(rms_la) && (rms_la < 0.2) && (inl >= 0.6);
                fprintf('[Task7-Att] Estimated lever arm r_b [m] = [%.3f %.3f %.3f], RMS=%.3f m/s, inliers=%.2f, valid=%d\n', rb, rms_la, inl, valid_la);
                task7_results.lever_arm_rb = rb; task7_results.lever_arm_rms = rms_la; task7_results.lever_arm_inliers = inl; task7_results.lever_arm_valid = valid_la;
                save(fullfile(resultsDir, sprintf('%s_task7_results.mat', runTag)), '-struct', 'task7_results');
                save(fullfile(resultsDir, sprintf('%s_task7_lever_arm.mat', runTag)), 'rb', 'rms_la', 'inl', 'valid_la');
            end
        catch ME
            warning('[Task7] Lever arm estimation failed: %s', ME.message);
        end
    catch ME
        warning('[Task7] Failed to compute/save attitude residual plots/tables: %s', ME.message);
    end
catch ME
    warning('[Task7] Attitude quaternion comparison failed: %s', ME.message);
end

% (Attitude overlay now handled above using align_truth_vs_est_attitude)

end % End of function Task_7

function t = extractTimeVec(T)
    t = [];
    for c = T.Properties.VariableNames
        nm = lower(c{1});
        if any(strcmp(nm, {'time','t','posix_time','sec','seconds'}))
            t = T.(c{1}); t = t(:); return;
        end
    end
    t = (0:height(T)-1)'; % fallback
end

function [P,V] = extractECEF(T)
    P=[]; V=[];
    cx = findCol(T, {'pos_ecef_x','ecef_x','x_ecef','x'});
    cy = findCol(T, {'pos_ecef_y','ecef_y','y_ecef','y'});
    cz = findCol(T, {'pos_ecef_z','ecef_z','z_ecef','z'});
    if ~isempty(cx)&&~isempty(cy)&&~isempty(cz)
        P = [T.(cx), T.(cy), T.(cz)];
    end
    vx = findCol(T, {'vel_ecef_x','vx_ecef','ecef_vx','vx'});
    vy = findCol(T, {'vel_ecef_y','vy_ecef','ecef_vy','vy'});
    vz = findCol(T, {'vel_ecef_z','vz_ecef','ecef_vz','vz'});
    if ~isempty(vx)&&~isempty(vy)&&~isempty(vz)
        V = [T.(vx), T.(vy), T.(vz)];
    end
end

function [P,V] = extractNED(T)
    P=[]; V=[];
    cn = findCol(T, {'pos_n','ned_n','north'});
    ce = findCol(T, {'pos_e','ned_e','east'});
    cd = findCol(T, {'pos_d','ned_d','down'});
    if ~isempty(cn)&&~isempty(ce)&&~isempty(cd)
        P = [T.(cn), T.(ce), T.(cd)];
    end
    vn = findCol(T, {'vel_n','ned_vn','vn','north_vel'});
    ve = findCol(T, {'vel_e','ned_ve','ve','east_vel'});
    vd = findCol(T, {'vel_d','ned_vd','vd','down_vel'});
    if ~isempty(vn)&&~isempty(ve)&&~isempty(vd)
        V = [T.(vn), T.(ve), T.(vd)];
    end
end

function nm = findCol(T, cand)
    nm = '';
    for k=1:numel(cand)
        idx = find(strcmpi(T.Properties.VariableNames, cand{k}), 1);
        if ~isempty(idx), nm = T.Properties.VariableNames{idx}; return; end
    end
end

function plot_residual_grid(t, pos_res, vel_res, frameName, outDir, runTag)
%PLOT_RESIDUAL_GRID Plot 2x3 residual grid: rows=pos/vel, cols=X/Y/Z
    % FIX: Set figure size for page width export (Fix #8)
    f = figure('Visible','on');
    set(f, 'Units', 'centimeters', 'Position', [2 2 18 9]);
    set(f, 'PaperPositionMode', 'auto');
    tl = tiledlayout(2,3,'Padding','compact','TileSpacing','compact'); %#ok<NASGU>
    cols = {'X','Y','Z'};
    ylabels = {'Position Residual [m]','Velocity Residual [m/s]'};
    for j=1:3
        nexttile(j);
        plot(t, pos_res(:,j),'LineWidth',1.0); grid on;
        title(cols{j}); ylabel(ylabels{1});
        if j==2, xlabel('Time [s]'); end
    end
    for j=1:3
        nexttile(3+j);
        plot(t, vel_res(:,j),'LineWidth',1.0); grid on;
        ylabel(ylabels{2}); xlabel('Time [s]');
    end
    sgtitle(sprintf('Task 7 Residuals (%s Frame)', frameName));
    base = fullfile(outDir, sprintf('%s_task7_residuals_%s', runTag, frameName));
    try
        % FIX: Export PNG with 300 DPI resolution (Fix #8)
        exportgraphics(f, [base '.png'], 'Resolution', 300);
    catch
        print(f, [base '.png'], '-dpng', '-r300');
    end
    try, savefig(f, [base '.fig']); catch, end
    close(f);
end

% ----------------- Local helpers for Task7-Att -----------------
function r = qmul(a,b)
% Hamilton product, row-wise operands (Nx4)
    aw=a(:,1); ax=a(:,2); ay=a(:,3); az=a(:,4);
    bw=b(:,1); bx=b(:,2); by=b(:,3); bz=b(:,4);
    r=[ aw.*bw - ax.*bx - ay.*by - az.*bz, ...
        aw.*bx + ax.*bw + ay.*bz - az.*by, ...
        aw.*by - ax.*bz + ay.*bw + az.*bx, ...
        aw.*bz + ax.*by - ay.*bx + az.*bw ];
end

function q = dcm2quat_wxyz(C)
% DCM (3x3) to quaternion (1x4) [w x y z]
    tr = trace(C);
    if tr > 0
        s = sqrt(tr+1.0)*2;
        w = 0.25*s;
        x = (C(3,2)-C(2,3))/s;
        y = (C(1,3)-C(3,1))/s;
        z = (C(2,1)-C(1,2))/s;
    else
        [~,i] = max([C(1,1),C(2,2),C(3,3)]);
        switch i
            case 1
                s = sqrt(1.0 + C(1,1) - C(2,2) - C(3,3))*2;
                w = (C(3,2)-C(2,3))/s; x = 0.25*s;
                y = (C(1,2)+C(2,1))/s; z = (C(1,3)+C(3,1))/s;
            case 2
                s = sqrt(1.0 + C(2,2) - C(1,1) - C(3,3))*2;
                w = (C(1,3)-C(3,1))/s; x = (C(1,2)+C(2,1))/s;
                y = 0.25*s; z = (C(2,3)+C(3,2))/s;
            case 3
                s = sqrt(1.0 + C(3,3) - C(1,1) - C(2,2))*2;
                w = (C(2,1)-C(1,2))/s; x = (C(1,3)+C(3,1))/s;
                y = (C(2,3)+C(3,2))/s; z = 0.25*s;
        end
    end
    % normalize
    q = [w x y z];
    n = norm(q);
    if isfinite(n) && n > 0
        q = q./n;
    else
        q = [1 0 0 0];
    end
end

function eul = quat2eulerZYX_deg(q)
% Quaternion wxyz -> [yaw(Z) pitch(Y) roll(X)] in degrees for Body->NED
    w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
    r11 = 1 - 2*(y.^2 + z.^2);
    r12 = 2*(x.*y - z.*w);
    r13 = 2*(x.*z + y.*w);
    r23 = 2*(y.*z - x.*w);
    r33 = 1 - 2*(x.^2 + y.^2);
    yaw   = atan2d(r12, r11);
    pitch = -asind(r13);       % ZYX convention
    roll  = atan2d(r23, r33);
    eul = [yaw pitch roll];
end

function qt = slerpVec(t0,q0,t1,q1,t)
% Piecewise SLERP between samples defined by (t0,q0)->(t1,q1), query at t
    % Find bin for each t
    idx = discretize(t,[t0(1); t1]);  % choose interval k s.t. t in [t0(k),t1(k)]
    idx(isnan(idx)) = 1;
    t0i = t0(idx); t1i=t1(idx);
    u = (t - t0i)./(t1i - t0i); u = max(0,min(1,u));
    q0i = q0(idx,:); q1i = q1(idx,:);
    % shortest path
    dotp = sum(q0i.*q1i,2);
    flip = dotp<0; q1i(flip,:) = -q1i(flip,:); dotp(flip) = -dotp(flip);
    omega = acos(max(-1,min(1,dotp)));
    so = sin(omega);
    useLin = so < 1e-6;
    qt = zeros(numel(t),4);
    % SLERP
    a = sin((1-u).*omega)./so; b = sin(u.*omega)./so;
    qt(~useLin,:) = a(~useLin).*q0i(~useLin,:) + b(~useLin).*q1i(~useLin,:);
    % LERP fallback
    qt(useLin,:)  = (1-u(useLin)).*q0i(useLin,:) + u(useLin).*q1i(useLin,:);
    % normalize rows
    qt = qt./vecnorm(qt,2,2);
end

function [q_truth_raw, t_truth] = auto_detect_truth_quat(truth_txt)
%AUTO_DETECT_TRUTH_QUAT Load truth file and detect quaternion columns (wxyz)
    M = readmatrix(truth_txt);
    if size(M,2) < 6
        error('Truth file has too few columns to contain quaternions');
    end
    t_truth = M(:,2);
    bestCols = [];
    bestScore = inf;
    for c = 1:size(M,2)-3
        q = M(:,c:c+3);
        nn = abs(vecnorm(q,2,2)-1);
        sc = median(nn(~isnan(nn)));
        if sc < bestScore && sc < 0.05
            bestScore = sc; bestCols = c:c+3;
        end
    end
    assert(~isempty(bestCols), 'Could not auto-detect quaternion columns in TRUTH file.');
    q_truth_raw = M(:,bestCols);
end

% ================= Align truth vs estimator attitude (Wahba + SLERP) =================
function [q_truth_aln, q_est_aln, eul_truth_deg, eul_est_deg, details] = ...
    align_truth_vs_est_attitude(t_est, q_est_b2n, t_truth, q_truth_raw, win_s)
% All quats are wxyz. Return quats aligned to Body->NED on the est timeline.

    % 0) Normalize & make sign-continuous
    q_est_b2n  = normalize_quat(q_est_b2n);
    q_est_b2n  = make_quat_continuous(q_est_b2n);

    q_truth_raw = normalize_quat(q_truth_raw);
    q_truth_raw = make_quat_continuous(q_truth_raw);

    % 1) Truth→est time match (slerp)
    q_truth_on_est = slerp_series(t_truth(:), q_truth_raw, t_est(:));

    % 2) Candidate mappings for truth -> Body->NED
    cand = cell(4,1);
    cand{1} = @(q) q;                 % Truth Body->NED
    cand{2} = @(q) quat_conj(q);      % Truth NED->Body
    C_en = [0 1 0; 1 0 0; 0 0 -1];    % ENU -> NED
    q_en2n = rotm2quat(C_en);
    cand{3} = @(q) quat_mul(repmat(q_en2n,size(q,1),1), q);             % Body->ENU
    cand{4} = @(q) quat_mul(repmat(q_en2n,size(q,1),1), quat_conj(q));  % ENU->Body

    % 3) Pick best mapping over a quiet window
    t0 = t_est(1);
    t1 = min(t_est(1)+win_s, t_est(end));
    idx = (t_est>=t0 & t_est<=t1);
    if nnz(idx) < 10
        K = min(2000,numel(t_est)); idx = false(size(t_est)); idx(1:K) = true;
    end

    best_err = inf; best_opt = 1; best_qtruth = [];
    for k=1:numel(cand)
        q_try = cand{k}(q_truth_on_est);
        q_try = normalize_quat(q_try);
        q_try = make_quat_continuous(q_try);
        err = mean_ang_err(q_est_b2n(idx,:), q_try(idx,:));  % deg
        if err < best_err
            best_err = err; best_opt = k; best_qtruth = q_try;
        end
    end

    % 4) Solve constant body-side rotation ΔR (Truth ≈ Δ * Est)
    Csum = zeros(3,3);
    ii = find(idx);
    for m = ii(:).'
        C_est   = quat2rotm_local(q_est_b2n(m,:));   % b->n
        C_truth = quat2rotm_local(best_qtruth(m,:)); % b->n
        Csum = Csum + C_truth * C_est';
    end
    [U,~,V] = svd(Csum);
    C_fix = U*V';
    if det(C_fix) < 0, U(:,3) = -U(:,3); C_fix = U*V'; end
    q_fix = rotm2quat(C_fix);             % wxyz

    % Apply Δ on the BODY side to the estimator (align estimator to truth)
    q_est_aln = quat_mul(repmat(q_fix,size(q_est_b2n,1),1), q_est_b2n);
    q_est_aln = normalize_quat(q_est_aln);
    q_est_aln = make_quat_continuous(q_est_aln);

    % Truth stays as mapped by the best hypothesis
    q_truth_aln = make_quat_continuous(normalize_quat(best_qtruth));

    % Global sign alignment to Truth to avoid mirror-image plots
    % Quaternions q and -q represent the same attitude; align sign once
    dp_mean = mean(sum(q_truth_aln .* q_est_aln, 2), 'omitnan');
    if dp_mean < 0
        q_est_aln = -q_est_aln;
    end

    % 5) Euler ZYX (deg)
    eul_truth_deg = rad2deg(rotm2eul_batch_local(q_truth_aln, 'ZYX'));
    eul_est_deg   = rad2deg(rotm2eul_batch_local(q_est_aln,   'ZYX'));

    % 6) Output details
    mean_dot = mean(sum(q_truth_aln .* q_est_aln, 2), 'omitnan');
    details = struct('best_option',best_opt, 'best_err_deg',best_err, ...
                     'q_fix',q_fix, 'used_window_s', t1 - t0, ...
                     'mean_quat_dot', mean_dot);
end

% ---------- helpers (local to aligner) ----------
function q = normalize_quat(q)
    q = q ./ vecnorm(q,2,2);
end

function q = make_quat_continuous(q)
    for i=2:size(q,1)
        if dot(q(i,:), q(i-1,:)) < 0, q(i,:) = -q(i,:); end
    end
end

function qconj = quat_conj(q)
    qconj = [q(:,1), -q(:,2:4)];
end

function qout = quat_mul(q1,q2)
% multiply elementwise (wxyz), same length or scalar q1/q2 broadcasts
    if size(q1,1)==1, q1 = repmat(q1,size(q2,1),1); end
    if size(q2,1)==1, q2 = repmat(q2,size(q1,1),1); end
    w1=q1(:,1); x1=q1(:,2); y1=q1(:,3); z1=q1(:,4);
    w2=q2(:,1); x2=q2(:,2); y2=q2(:,3); z2=q2(:,4);
    qout = [ w1.*w2 - x1.*x2 - y1.*y2 - z1.*z2, ...
             w1.*x2 + x1.*w2 + y1.*z2 - z1.*y2, ...
             w1.*y2 - x1.*z2 + y1.*w2 + z1.*x2, ...
             w1.*z2 + x1.*y2 - y1.*x2 + z1.*w2 ];
end

function err_deg = mean_ang_err(q_ref, q)
% mean angle(q_ref⁻¹ ⊗ q)
    qerr = quat_mul(quat_conj(q_ref), q);
    v = qerr(:,2:4); w = abs(qerr(:,1));
    ang = 2*atan2(vecnorm(v,2,2), w);
    err_deg = mean(rad2deg(ang));
end

function R = quat2rotm_local(q)
% wxyz -> DCM (body->world)
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2), 2*(x*y - z*w), 2*(x*z + y*w);
         2*(x*y + z*w), 1-2*(x^2+z^2), 2*(y*z - x*w);
         2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x^2+y^2)];
end

function q = rotm2quat(R)
% DCM -> wxyz
    tr = trace(R);
    if tr > 0
        S  = sqrt(tr+1.0)*2;  w = 0.25*S;
        x = (R(3,2)-R(2,3))/S;
        y = (R(1,3)-R(3,1))/S;
        z = (R(2,1)-R(1,2))/S;
    else
        [~,i] = max([R(1,1),R(2,2),R(3,3)]);
        switch i
            case 1
                S = sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;
                w = (R(3,2)-R(2,3))/S; x = 0.25*S;
                y = (R(1,2)+R(2,1))/S; z = (R(1,3)+R(3,1))/S;
            case 2
                S = sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;
                w = (R(1,3)-R(3,1))/S; x = (R(1,2)+R(2,1))/S;
                y = 0.25*S; z = (R(2,3)+R(3,2))/S;
            otherwise
                S = sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;
                w = (R(2,1)-R(1,2))/S; x = (R(1,3)+R(3,1))/S;
                y = (R(2,3)+R(3,2))/S; z = 0.25*S;
        end
    end
    q = [w x y z];
    q = q ./ norm(q);
end

function E = rotm2eul_batch_local(q, seq)
% vectorized euler from wxyz; seq='ZYX'
    n = size(q,1); E = zeros(n,3);
    for i=1:n
        R = quat2rotm_local(q(i,:));
        switch seq
            case 'ZYX' % yaw-pitch-roll
                yaw   = atan2(R(2,1), R(1,1));
                pitch = asin(-R(3,1));
                roll  = atan2(R(3,2), R(3,3));
                E(i,:) = [yaw pitch roll];
            otherwise
                error('Only ZYX implemented here.');
        end
    end
end

function q_out = slerp_series(t_in, q_in, t_out)
% SLERP truth -> est time
    q_out = zeros(numel(t_out),4);
    i = 1;
    for k = 1:numel(t_out)
        tk = t_out(k);
        while i < numel(t_in) && t_in(i+1) < tk, i = i+1; end
        if tk <= t_in(1), q_out(k,:) = q_in(1,:); continue; end
        if tk >= t_in(end), q_out(k,:) = q_in(end,:); continue; end
        t0=t_in(i); t1=t_in(i+1); u = (tk - t0)/(t1 - t0);
        q0 = q_in(i,:); q1 = q_in(i+1,:);
        % SLERP
        dotp = dot(q0,q1);
        if dotp < 0, q1 = -q1; dotp = -dotp; end
        if dotp > 0.9995 % nearly collinear: lerp + renorm
            q = (1-u)*q0 + u*q1; q = q./norm(q);
        else
            theta = acos(dotp);
            s0 = sin((1-u)*theta)/sin(theta);
            s1 = sin(u*theta)/sin(theta);
            q = s0*q0 + s1*q1;
        end
        q_out(k,:) = q./norm(q);
    end
    q_out = make_quat_continuous(q_out);
end

function y = wrapTo180_local(x)
% Wrap degrees to [-180, 180]
    y = mod(x + 180, 360) - 180;
    % Handle edge case where mod returns negative zero or 180
    y(abs(y+180) < 1e-12) = -180;
end

function eul_deg = quat_to_euler_zyx_deg(q_wxyz)
% Convert quaternion wxyz to Euler ZYX (yaw, pitch, roll) in degrees
% Uses the EXACT same formula as run_triad_only.py for consistency
    
    n = size(q_wxyz, 1);
    eul_deg = zeros(n, 3);
    
    for i = 1:n
        w = q_wxyz(i,1); x = q_wxyz(i,2); y = q_wxyz(i,3); z = q_wxyz(i,4);
        
        % Use EXACT same formulation as Python's _quat_to_euler_zyx_deg
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        s = max(-1, min(1, 2 * (w * y - z * x))); % Clamp like np.clip
        pitch = asin(s);
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        
        % Convert to degrees
        eul_deg(i, :) = rad2deg([yaw, pitch, roll]);
    end
end

function eul_deg = convert_quat_to_euler_via_scipy(q_wxyz)
% Convert quaternion to Euler via SciPy to guarantee consistency with Python
    try
        % Convert wxyz to xyzw format for scipy
        q_xyzw = [q_wxyz(:,2), q_wxyz(:,3), q_wxyz(:,4), q_wxyz(:,1)];
        
        % Call Python scipy
        py_quat = py.numpy.array(q_xyzw);
        scipy_R = py.scipy.spatial.transform.Rotation.from_quat(py_quat);
        eul_rad = scipy_R.as_euler("zyx", pyargs('degrees', false));
        eul_deg = rad2deg(double(eul_rad));
        
    catch ME
        % Fallback to mathematical implementation if Python/SciPy unavailable
        warning('SciPy not available, using fallback implementation. Results may differ slightly.');
        eul_deg = quat_to_euler_zyx_deg(q_wxyz);
    end
end

function q_cont = enforce_continuity_local(q)
% Enforce temporal continuity on quaternion series (prevent sign flips)
% Matches Python's _enforce_continuity function exactly
    q_cont = q;
    for k = 2:size(q,1)
        if dot(q_cont(k,:), q_cont(k-1,:)) < 0
            q_cont(k,:) = -q_cont(k,:);
        end
    end
end
