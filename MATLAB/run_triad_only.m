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
if ~isfield(cfg,'vel_q_scale'), cfg.vel_q_scale = 10.0; end
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

% Task 6 can accept either the Task 5 .mat or (imu,gnss,truth) paths — use the version you have:
if exist('Task_6.m','file')
    Task_6(fullfile(cfg.paths.matlab_results, sprintf('%s_task5_results.mat', rid)), ...
           cfg.imu_path, cfg.gnss_path, cfg.truth_path);
end

if exist('Task_7.m','file')
    Task_7();
end

fprintf('TRIAD processing complete for %s\n', cfg.dataset_id);
end
