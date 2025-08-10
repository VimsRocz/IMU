function run_triad_only(cfg)
% RUN_TRIAD_ONLY  Process dataset using TRIAD (Tasks 1..7) — MATLAB-only, no Python deps.
%
% Usage:
%   run_triad_only();
%   run_triad_only(struct(''dataset_id'',''X002''));

% ---- paths / utils ----
% Ensure utils are on the path before calling project_paths
here = fileparts(mfilename('fullpath'));  % .../MATLAB
utils_candidates = {
    fullfile(here,'utils'),
    fullfile(here,'src','utils'),
    fullfile(here,'src','utils','attitude'),
    fullfile(here,'src','utils','frames')
};
for i = 1:numel(utils_candidates)
    p = utils_candidates{i};
    if exist(p,'dir') && ~contains(path, [p pathsep])
        addpath(p);
    end
end
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
if ~isfield(cfg.plots,'popup_figures'), cfg.plots.popup_figures = false; end
if ~isfield(cfg.plots,'save_pdf'),      cfg.plots.save_pdf      = true;  end
if ~isfield(cfg.plots,'save_png'),      cfg.plots.save_png      = true;  end

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
Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);

% Task 6 can accept either the Task 5 .mat or (imu,gnss,truth) paths — use the version you have:
if exist('Task_6.m','file')
    try
        Task_6([], cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    catch
        % fallback signature
        Task_6(fullfile(cfg.paths.matlab_results, sprintf('%s_task5_results.mat', rid)), cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    end
end

if exist('Task_7.m','file')
    Task_7();
end

fprintf('TRIAD processing complete for %s\n', cfg.dataset_id);
end
