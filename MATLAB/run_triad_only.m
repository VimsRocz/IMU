 function run_triad_only(cfg)
% RUN_TRIAD_ONLY  Process dataset using TRIAD (Tasks 1..7) — MATLAB-only, no Python deps.
%
% Usage:
%   run_triad_only();
%   run_triad_only(struct(''dataset_id'',''X002''));

% Resolve repo root from MATLAB/ folder
thisFile  = mfilename('fullpath');
matlabDir = fileparts(thisFile);
repoRoot  = fileparts(matlabDir);

dataDir   = fullfile(repoRoot, 'DATA');
imuDir    = fullfile(dataDir, 'IMU');
gnssDir   = fullfile(dataDir, 'GNSS');
truthDir  = fullfile(dataDir, 'Truth');

matResDir = fullfile(matlabDir, 'results');
if ~exist(matResDir,'dir'); mkdir(matResDir); end
addpath(genpath(fullfile(matlabDir,'src')));   % MATLAB utilities
addpath(genpath(fullfile(matlabDir,'utils'))); % extra helpers

paths = struct('root', repoRoot, 'matlab_results', matResDir);
if nargin==0 || isempty(cfg), cfg = struct(); end

% ---- config (explicit, no hidden defaults) ----
if ~isfield(cfg,'dataset_id'), cfg.dataset_id = 'X002'; end
if ~isfield(cfg,'method'),     cfg.method     = 'TRIAD'; end
if ~isfield(cfg,'imu_file'),   cfg.imu_file   = 'IMU_X002.dat'; end
if ~isfield(cfg,'gnss_file'),  cfg.gnss_file  = 'GNSS_X002.csv'; end
% Single-truth-file policy (always this name)
if ~isfield(cfg,'truth_file'), cfg.truth_file = 'STATE_X001.txt'; end

% Plot options defaults to avoid missing-field errors in Task 4/5
if ~isfield(cfg,'plots') || ~isstruct(cfg.plots)
    cfg.plots = struct();
end
if ~isfield(cfg.plots,'popup_figures'), cfg.plots.popup_figures = true; end
if ~isfield(cfg.plots,'save_pdf'),      cfg.plots.save_pdf      = false; end
if ~isfield(cfg.plots,'save_png'),      cfg.plots.save_png      = false; end
% KF tuning defaults (safe if default_cfg not reloaded)
if ~isfield(cfg,'vel_q_scale'), cfg.vel_q_scale = 1.0; end
if ~isfield(cfg,'vel_r'),       cfg.vel_r       = 0.25; end
% Optional auto-tune flag
if ~isfield(cfg,'autotune'),    cfg.autotune    = true; end
% Optional trace capture (first N KF steps)
if ~isfield(cfg,'trace_first_n'), cfg.trace_first_n = 0; end

cfg.paths = paths;

% ---- resolve inputs using DATA/* structure ----
cfg.imu_path   = fullfile(imuDir,   cfg.imu_file);
cfg.gnss_path  = fullfile(gnssDir,  cfg.gnss_file);
cfg.truth_path = fullfile(truthDir, cfg.truth_file);
if ~isfile(cfg.imu_path)
    error('IMU file not found: %s', cfg.imu_path);
end
if ~isfile(cfg.gnss_path)
    error('GNSS file not found: %s', cfg.gnss_path);
end
if ~isfile(cfg.truth_path)
    warning('Truth file missing at %s', cfg.truth_path);
end

% ---- run id + timeline (before tasks) ----
rid = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);
print_timeline_matlab(rid, cfg.imu_path, cfg.gnss_path, cfg.truth_path, cfg.paths.matlab_results);
% Also load and print TRUTH timeline via robust loader
try
    if isfile(cfg.truth_path)
        load_truth_file(cfg.truth_path); % prints a TRUTH | ... line
    end
catch ME
    warning('Truth load failed: %s', ME.message);
end

fprintf('▶ %s\n', rid);
fprintf('MATLAB results dir: %s\n', cfg.paths.matlab_results);

% ---- Tasks 1..7 (compulsory) ----
Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
% Optionally auto-tune Q/R on a small grid before the final full run
if cfg.autotune
    fprintf('Auto-tune sweep over vel_q_scale and vel_r...\n');
    grid_q = [5, 10, 20, 40];
    grid_r = [0.25, 0.5, 1.0];
    rows = zeros(numel(grid_q)*numel(grid_r), 3);
    idx = 1;
    for iq = 1:numel(grid_q)
        for ir = 1:numel(grid_r)
            qv = grid_q(iq); rv = grid_r(ir);
            rmse = Task_5_try_once(cfg, qv, rv);
            rows(idx,:) = [qv, rv, rmse];
            idx = idx + 1;
        end
    end
    % Print table
    fprintf('\n q_scale    vel_r    rmse_pos\n');
    for i = 1:size(rows,1)
        fprintf(' %7.3f  %7.3f  %8.3f\n', rows(i,1), rows(i,2), rows(i,3));
    end
    % Pick best (min rmse)
    [~,j] = min(rows(:,3));
    if isfinite(rows(j,3))
        cfg.vel_q_scale = rows(j,1);
        cfg.vel_r       = rows(j,2);
        fprintf('Auto-tune best: vel_q_scale=%.3f  vel_r=%.3f  (RMSE_pos=%.3f m)\n', rows(j,1), rows(j,2), rows(j,3));
    else
        warning('Auto-tune produced no valid result; keeping defaults.');
    end
end

Task_5(cfg.imu_path, cfg.gnss_path, cfg.method, [], ...
       'vel_q_scale', cfg.vel_q_scale, 'vel_r', cfg.vel_r, 'trace_first_n', cfg.trace_first_n);

runTag = rid; resultsDir = cfg.paths.matlab_results; dataTruthDir = truthDir;

disp('Starting Task 6 overlay ...');

% Load Task-5 results (must contain time vector and, if available, q_b2n[t])
res5_file = fullfile(resultsDir, sprintf('%s_task5_results.mat', runTag));
fprintf('[Task6] Loading Task5 results from %s\n', res5_file);
res5 = load(res5_file);
% Expected available:
%  t_est [N x 1], pos_ned_est/vel_ned_est/acc_ned_est [N x 3] OR equivalents
%  pos_ecef_est/vel_ecef_est/acc_ecef_est [N x 3] (if stored)
%  q_b2n_hist [N x 4] (wxyz)  <-- optional but preferred

% Require estimator attitude for Task-6 overlays that include attitude/Euler
if ~isfield(res5, 'att_quat')
    error('Task6: att_quat missing from Task5 results; cannot plot attitude overlays.');
else
    q_est = res5.att_quat; % [N x 4], wxyz, Body->NED
    fprintf('[Task6] Using estimator attitude from att_quat (Nx4, wxyz, Body->NED). Size=%dx%d\n', size(q_est,1), size(q_est,2));
    if size(q_est,1) >= 2
        fprintf('[Task6] att_quat first=[% .4f % .4f % .4f % .4f], last=[% .4f % .4f % .4f % .4f]\n', ...
            q_est(1,1), q_est(1,2), q_est(1,3), q_est(1,4), q_est(end,1), q_est(end,2), q_est(end,3), q_est(end,4));
    end
end

% Time base
if isfield(res5,'t_est'), t_est = res5.t_est; else, error('Task6: t_est missing'); end
N = numel(t_est);
fprintf('[Task6] Time base: t_est length=%d (%.2f s span)\n', N, t_est(end) - t_est(1));

% Estimated NED triplets (build if missing from Task-4/5 cached signals)
EST = struct('pos_ned',[],'vel_ned',[],'acc_ned',[], ...
             'pos_ecef',[],'vel_ecef',[],'acc_ecef',[]);
copyIf = @(S,fn) (isfield(S,fn) && ~isempty(S.(fn)));
if copyIf(res5,'pos_ned_est'), EST.pos_ned = res5.pos_ned_est; end
if copyIf(res5,'vel_ned_est'), EST.vel_ned = res5.vel_ned_est; end
if copyIf(res5,'acc_ned_est'), EST.acc_ned = res5.acc_ned_est; end
if copyIf(res5,'pos_ecef_est'), EST.pos_ecef = res5.pos_ecef_est; end
if copyIf(res5,'vel_ecef_est'), EST.vel_ecef = res5.vel_ecef_est; end
if copyIf(res5,'acc_ecef_est'), EST.acc_ecef = res5.acc_ecef_est; end
fprintf('[Task6] Fields present | NED: pos=%s vel=%s acc=%s | ECEF: pos=%s vel=%s acc=%s\n', ...
    mat2str(~isempty(EST.pos_ned)), mat2str(~isempty(EST.vel_ned)), mat2str(~isempty(EST.acc_ned)), ...
    mat2str(~isempty(EST.pos_ecef)), mat2str(~isempty(EST.vel_ecef)), mat2str(~isempty(EST.acc_ecef)));

% Truth loading (prefer robust STATE_X parser)
truthPath = fullfile(dataTruthDir, 'STATE_X001.txt');
fprintf('[Task6] Loading TRUTH from %s\n', truthPath);
pos_ecef_truth = []; vel_ecef_truth = [];
try
    truth_data = read_state_file(truthPath);
    if size(truth_data,2) >= 8
        t_truth = truth_data(:,2);
        pos_ecef_truth = truth_data(:,3:5);
        vel_ecef_truth = truth_data(:,6:8);
    else
        error('STATE file missing columns');
    end
catch
    % Fallback to header-based table extraction
    T = readtable(truthPath, detectImportOptions(truthPath));
    t_truth = extractTimeVec(T);
    [pos_ecef_truth, vel_ecef_truth] = extractECEF(T);
end
pos_ned_truth = []; vel_ned_truth = [];
fprintf('[Task6] Truth sizes | t=%d pos_ecef=%dx%d vel_ecef=%dx%d pos_ned=%dx%d vel_ned=%dx%d\n', ...
    numel(t_truth), size(pos_ecef_truth,1), size(pos_ecef_truth,2), size(vel_ecef_truth,1), size(vel_ecef_truth,2), ...
    size(pos_ned_truth,1), size(pos_ned_truth,2), size(vel_ned_truth,1), size(vel_ned_truth,2));

% lat/lon from Task-1/4
if isfield(res5,'ref_lat'), computed_lat_rad = res5.ref_lat; else, computed_lat_rad = 0; end
if isfield(res5,'ref_lon'), computed_lon_rad = res5.ref_lon; else, computed_lon_rad = 0; end
lat = computed_lat_rad; lon = computed_lon_rad;  % already computed earlier
fprintf('[Task6] Reference lat/lon (deg): %.6f / %.6f\n', rad2deg(lat), rad2deg(lon));

% Ensure NED truth (convert from ECEF if needed)
if isempty(pos_ned_truth) && ~isempty(pos_ecef_truth)
    fprintf('[Task6] Converting TRUTH position ECEF->NED...\n');
    % Subtract reference ECEF origin so NED positions are relative to r0
    r0 = zeros(1,3);
    if isfield(res5,'ref_r0') && ~isempty(res5.ref_r0)
        r0 = res5.ref_r0(:)';
    end
    pos_ned_truth = attitude_tools('ecef2ned_vec', pos_ecef_truth - r0, lat, lon);
end
if isempty(vel_ned_truth) && ~isempty(vel_ecef_truth)
    fprintf('[Task6] Converting TRUTH velocity ECEF->NED...\n');
    vel_ned_truth = attitude_tools('ecef2ned_vec', vel_ecef_truth, lat, lon);
end

% Interp truth to estimator time
interp = @(X) attitude_tools('interp_to', t_truth, X, t_est);
TRU_NED.pos = interp(pos_ned_truth);
TRU_NED.vel = interp(vel_ned_truth);
TRU_NED.acc = [];  % optional in truth
fprintf('[Task6] Interpolated TRUTH to estimator time | NED pos=%dx%d vel=%dx%d\n', size(TRU_NED.pos), size(TRU_NED.vel));

% Build EST NED (convert from ECEF if missing)
if isempty(EST.pos_ned) && ~isempty(EST.pos_ecef)
    fprintf('[Task6] Converting EST position ECEF->NED...\n');
    r0 = zeros(1,3);
    if isfield(res5,'ref_r0') && ~isempty(res5.ref_r0)
        r0 = res5.ref_r0(:)';
    end
    EST.pos_ned = attitude_tools('ecef2ned_vec', EST.pos_ecef - r0, lat, lon);
end
if isempty(EST.vel_ned) && ~isempty(EST.vel_ecef)
    fprintf('[Task6] Converting EST velocity ECEF->NED...\n');
    EST.vel_ned = attitude_tools('ecef2ned_vec', EST.vel_ecef, lat, lon);
end
if isempty(EST.acc_ned) && ~isempty(EST.acc_ecef)
    fprintf('[Task6] Converting EST acceleration ECEF->NED...\n');
    EST.acc_ned = attitude_tools('ecef2ned_vec', EST.acc_ecef, lat, lon);
end

EST_NED.pos = EST.pos_ned; EST_NED.vel = EST.vel_ned; EST_NED.acc = EST.acc_ned;
fprintf('[Task6] EST NED sizes | pos=%dx%d vel=%dx%d acc=%dx%d\n', size(EST_NED.pos), size(EST_NED.vel), size(EST_NED.acc));

% Build ECEF triplets (prefer native; else convert from NED)
if isempty(EST.pos_ecef) && ~isempty(EST.pos_ned)
    fprintf('[Task6] Converting EST position NED->ECEF...\n');
    EST.pos_ecef = attitude_tools('ned2ecef_vec', EST.pos_ned, lat, lon);
end
if isempty(EST.vel_ecef) && ~isempty(EST.vel_ned)
    fprintf('[Task6] Converting EST velocity NED->ECEF...\n');
    EST.vel_ecef = attitude_tools('ned2ecef_vec', EST.vel_ned, lat, lon);
end
if isempty(EST.acc_ecef) && ~isempty(EST.acc_ned)
    fprintf('[Task6] Converting EST acceleration NED->ECEF...\n');
    EST.acc_ecef = attitude_tools('ned2ecef_vec', EST.acc_ned, lat, lon);
end
TRU_ECEF.pos = interp(pos_ecef_truth);
TRU_ECEF.vel = interp(vel_ecef_truth);
TRU_ECEF.acc = [];
fprintf('[Task6] TRUTH ECEF sizes | pos=%dx%d vel=%dx%d\n', size(TRU_ECEF.pos), size(TRU_ECEF.vel));

% Persist converted truth for downstream inspection
try
    truth_conv_file = fullfile(fullfile(resultsDir, runTag), sprintf('%s_task6_truth_converted.mat', runTag));
    try
        pos_body_truth = TRU_BODY.pos; vel_body_truth = TRU_BODY.vel; acc_body_truth = TRU_BODY.acc; %#ok<NASGU>
    catch
        pos_body_truth = []; vel_body_truth = []; acc_body_truth = [];
    end
    save(truth_conv_file, 't_truth', 'pos_ecef_truth', 'vel_ecef_truth', 'pos_ned_truth', 'vel_ned_truth', ...
                           'pos_body_truth', 'vel_body_truth', 'acc_body_truth');
    fprintf('[Task6] Saved converted TRUTH to %s\n', truth_conv_file);
catch ME
    warning('[Task6] Failed to save converted TRUTH: %s', ME.message);
end

% BODY with time-varying q_b2n[t] if available
q_hist = [];
if isfield(res5,'q_b2n_hist'), q_hist = res5.q_b2n_hist; end
if ~isempty(q_hist)
    if size(q_hist,1)==N && size(q_hist,2)==4, q_hist = q_hist.'; end % make [4 x N]
    q_hist = attitude_tools('quat_hemi', attitude_tools('quat_normalize', q_hist));
else
    % fallback: Task-3 constant quaternion if stored, else identity
    if exist(fullfile(resultsDir, sprintf('%s_task3_results.mat', runTag)), 'file')
        r3 = load(fullfile(resultsDir, sprintf('%s_task3_results.mat', runTag)));
        if isfield(r3,'q_triad'), q_hist = repmat(r3.q_triad(:),1,N); end
    end
    if isempty(q_hist), q_hist = repmat([1;0;0;0],1,N); end
end

TRU_BODY.pos = []; TRU_BODY.vel = []; TRU_BODY.acc = [];
if ~isempty(EST_NED.pos), EST_BODY.pos = attitude_tools('ned2body_series', EST_NED.pos, q_hist); else, EST_BODY.pos = []; end
if ~isempty(EST_NED.vel), EST_BODY.vel = attitude_tools('ned2body_series', EST_NED.vel, q_hist); else, EST_BODY.vel = []; end
if ~isempty(EST_NED.acc), EST_BODY.acc = attitude_tools('ned2body_series', EST_NED.acc, q_hist); else, EST_BODY.acc = []; end

if ~isempty(TRU_NED.pos), TRU_BODY.pos = attitude_tools('ned2body_series', TRU_NED.pos, q_hist); end
if ~isempty(TRU_NED.vel), TRU_BODY.vel = attitude_tools('ned2body_series', TRU_NED.vel, q_hist); end
if ~isempty(TRU_NED.acc), TRU_BODY.acc = attitude_tools('ned2body_series', TRU_NED.acc, q_hist); end
fprintf('[Task6] BODY sizes | EST pos=%dx%d vel=%dx%d acc=%dx%d | TRU pos=%dx%d vel=%dx%d acc=%dx%d\n', ...
    size(EST_BODY.pos), size(EST_BODY.vel), size(EST_BODY.acc), size(TRU_BODY.pos), size(TRU_BODY.vel), size(TRU_BODY.acc));

% 3×3 overlays (PNG + FIG), matching Task 5 naming
out6_dir = fullfile(resultsDir, runTag);
if ~exist(out6_dir,'dir'), mkdir(out6_dir); end

fprintf('[Task6] Plotting overlay grids (NED/ECEF/BODY)...\n');
% NED
fused_ned = struct('t', t_est, 'pos', EST_NED.pos, 'vel', EST_NED.vel, 'acc', EST_NED.acc);
truth_ned = struct('t', t_est, 'pos', TRU_NED.pos, 'vel', TRU_NED.vel, 'acc', TRU_NED.acc);
h_ned = plot_state_grid_overlay(t_est, fused_ned, truth_ned, 'NED', 'Title', [runTag ' Task6: Fused vs Truth'], 'Visible', 'on', 'MaxPlotPoints', 20000);
set(h_ned, 'PaperPositionMode','auto');
ned_grid_png = fullfile(out6_dir, sprintf('%s_task6_overlay_grid_NED.png', runTag));
ned_grid_fig = fullfile(out6_dir, sprintf('%s_task6_overlay_grid_NED.fig', runTag));
print(h_ned, strrep(ned_grid_png,'.png','.pdf'), '-dpdf', '-bestfit');
try, exportgraphics(h_ned, ned_grid_png, 'Resolution', 300); catch, print(h_ned, ned_grid_png, '-dpng', '-r300'); end
try, savefig(h_ned, ned_grid_fig); catch, end
% Task5-style name
ned_state_png = fullfile(resultsDir, sprintf('%s_task6_NED_state.png', runTag));
try, exportgraphics(h_ned, ned_state_png, 'Resolution', 300); catch, print(h_ned, ned_state_png, '-dpng', '-r300'); end
try, savefig(h_ned, strrep(ned_state_png,'.png','.fig')); catch, end

% ECEF
fused_ecef = struct('t', t_est, 'pos', EST.pos_ecef, 'vel', EST.vel_ecef, 'acc', EST.acc_ecef);
truth_ecef = struct('t', t_est, 'pos', TRU_ECEF.pos, 'vel', TRU_ECEF.vel, 'acc', TRU_ECEF.acc);
h_ecef = plot_state_grid_overlay(t_est, fused_ecef, truth_ecef, 'ECEF', 'Title', [runTag ' Task6: Fused vs Truth'], 'Visible', 'on', 'MaxPlotPoints', 20000);
set(h_ecef, 'PaperPositionMode','auto');
ecef_grid_png = fullfile(out6_dir, sprintf('%s_task6_overlay_grid_ECEF.png', runTag));
ecef_grid_fig = fullfile(out6_dir, sprintf('%s_task6_overlay_grid_ECEF.fig', runTag));
print(h_ecef, strrep(ecef_grid_png,'.png','.pdf'), '-dpdf', '-bestfit');
try, exportgraphics(h_ecef, ecef_grid_png, 'Resolution', 300); catch, print(h_ecef, ecef_grid_png, '-dpng', '-r300'); end
try, savefig(h_ecef, ecef_grid_fig); catch, end
ecef_state_png = fullfile(resultsDir, sprintf('%s_task6_ECEF_state.png', runTag));
try, exportgraphics(h_ecef, ecef_state_png, 'Resolution', 300); catch, print(h_ecef, ecef_state_png, '-dpng', '-r300'); end
try, savefig(h_ecef, strrep(ecef_state_png,'.png','.fig')); catch, end

% BODY
fused_body = struct('t', t_est, 'pos', EST_BODY.pos, 'vel', EST_BODY.vel, 'acc', EST_BODY.acc);
truth_body = struct('t', t_est, 'pos', TRU_BODY.pos, 'vel', TRU_BODY.vel, 'acc', TRU_BODY.acc);
h_body = plot_state_grid_overlay(t_est, fused_body, truth_body, 'Body', 'Title', [runTag ' Task6: Fused vs Truth'], 'Visible', 'on', 'MaxPlotPoints', 20000);
set(h_body, 'PaperPositionMode','auto');
body_grid_png = fullfile(out6_dir, sprintf('%s_task6_overlay_grid_Body.png', runTag));
body_grid_fig = fullfile(out6_dir, sprintf('%s_task6_overlay_grid_Body.fig', runTag));
print(h_body, strrep(body_grid_png,'.png','.pdf'), '-dpdf', '-bestfit');
try, exportgraphics(h_body, body_grid_png, 'Resolution', 300); catch, print(h_body, body_grid_png, '-dpng', '-r300'); end
try, savefig(h_body, body_grid_fig); catch, end
body_state_png = fullfile(resultsDir, sprintf('%s_task6_BODY_state.png', runTag));
try, exportgraphics(h_body, body_state_png, 'Resolution', 300); catch, print(h_body, body_state_png, '-dpng', '-r300'); end
try, savefig(h_body, strrep(body_state_png,'.png','.fig')); catch, end

fprintf('[Task6] Saved overlays:\n  %s\n  %s\n  %s\n', ned_grid_png, ecef_grid_png, body_grid_png);

quatType = 'constant';
if ~isempty(q_hist) && size(unique(q_hist.', 'rows'),1) > 1
    quatType = 'time-varying';
end
fprintf('[Task6] Using %s quaternion: %d\n', quatType, ~isempty(q_hist));

if exist('Task_7.m','file')
    Task_7();
end

fprintf('TRIAD processing complete for %s\n', cfg.dataset_id);
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
