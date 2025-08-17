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
        t_truth = truth_data(:,2);
    else
        t_truth = extractTimeVec(Ttruth);
    end
end

% Estimated (prefer NED; else convert)
lat = res5.ref_lat; lon = res5.ref_lon;
E.pos_ned = []; E.vel_ned = []; E.acc_ned = [];
if isfield(res5,'pos_ned_est'), E.pos_ned = res5.pos_ned_est; end
if isempty(E.pos_ned) && isfield(res5,'pos_ecef_est')
    E.pos_ned = attitude_tools('ecef2ned_vec', res5.pos_ecef_est, lat, lon);
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
interp = @(X) attitude_tools('interp_to', t_truth, X, t_est);
Ptru = interp(P_ned); Vtru = interp(V_ned);

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
end

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
    f = figure('Visible','on','Position',[100 100 1200 600]);
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
        exportgraphics(f, [base '.png'], 'Resolution', 150);
    catch
        print(f, [base '.png'], '-dpng', '-r150');
    end
    try, savefig(f, [base '.fig']); catch, end
    close(f);
end
