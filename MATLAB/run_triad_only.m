function run_triad_only(cfg)
% RUN_TRIAD_ONLY  Process dataset using TRIAD (Tasks 1..7).
%
% Usage:
%   run_triad_only();
%   run_triad_only(struct('dataset_id','X002'));

addpath(genpath(fullfile(pwd,'MATLAB','src','utils')));
set_debug(strcmpi(getenv('DEBUG'),'1') || strcmpi(getenv('DEBUG'),'true'));
log_msg('[BOOT] run_triad_only.m loaded');
paths = project_paths();  % adds utils; returns root/matlab/results
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
if ~isfield(cfg.plots,'save_pdf'),      cfg.plots.save_pdf      = true;  end
if ~isfield(cfg.plots,'save_png'),      cfg.plots.save_png      = true;  end
% KF tuning defaults (safe if default_cfg not reloaded)
if ~isfield(cfg,'vel_q_scale'), cfg.vel_q_scale = 10.0; end
if ~isfield(cfg,'vel_r'),       cfg.vel_r       = 0.25; end
% Optional auto-tune flag
if ~isfield(cfg,'autotune'),    cfg.autotune    = true; end
% Optional trace capture (first N KF steps)
if ~isfield(cfg,'trace_first_n'), cfg.trace_first_n = 0; end

cfg.paths = paths;

% ---- resolve inputs (copy to root if found elsewhere) ----
cfg.imu_path   = ensure_input_file('IMU',   cfg.imu_file,   cfg.paths);
cfg.gnss_path  = ensure_input_file('GNSS',  cfg.gnss_file,  cfg.paths);
cfg.truth_path = ensure_input_file('TRUTH', cfg.truth_file, cfg.paths);

% ---- run id + timeline (before tasks) ----
rid = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);
print_timeline_matlab(rid, cfg.imu_path, cfg.gnss_path, cfg.truth_path, cfg.paths.matlab_results);

fprintf('▶ %s\n', rid);
fprintf('MATLAB results dir: %s\n', cfg.paths.matlab_results);

% ---- Tasks 1..7 (compulsory) ----
try_task('Task_1', @Task_1, cfg.imu_path, cfg.gnss_path, cfg.method);
try_task('Task_2', @Task_2, cfg.imu_path, cfg.gnss_path, cfg.method);
try_task('Task_3', @Task_3, cfg.imu_path, cfg.gnss_path, cfg.method);
try_task('Task_4', @Task_4, cfg.imu_path, cfg.gnss_path, cfg.method);
% Optionally auto-tune Q/R on a small grid before the final full run
if cfg.autotune
    try
        grid_q = [5, 10, 20, 40];
        grid_r = [0.25, 0.5, 1.0];
        [best_q, best_r, report] = task5_autotune(cfg.imu_path, cfg.gnss_path, cfg.method, grid_q, grid_r);
        fprintf('Auto-tune best: vel_q_scale=%.3f  vel_r=%.3f  (RMSE_pos=%.3f m)\n', report.best_rmse_q, report.best_rmse_r, report.best_rmse);
        cfg.vel_q_scale = best_q; cfg.vel_r = best_r;
    catch ME
        warning('Auto-tune failed: %s. Proceeding with defaults.', ME.message);
    end
end

try_task('Task_5', @Task_5, cfg.imu_path, cfg.gnss_path, cfg.method, [], ...
       'vel_q_scale', cfg.vel_q_scale, 'vel_r', cfg.vel_r, 'trace_first_n', cfg.trace_first_n);

% Task 6 can accept either the Task 5 .mat or (imu,gnss,truth) paths — use the version you have:
if exist('Task_6.m','file')
    try_task('Task_6', @Task_6, fullfile(cfg.paths.matlab_results, sprintf('%s_task5_results.mat', rid)), ...
           cfg.imu_path, cfg.gnss_path, cfg.truth_path);
end

if exist('Task_7.m','file')
    try_task('Task_7', @Task_7);
end

fprintf('TRIAD processing complete for %s\n', cfg.dataset_id);
end
