function run_triad_only(cfg)
% ensure utils on path
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src','utils'));
% RUN_TRIAD_ONLY  Process dataset using TRIAD (Tasks 1..7).
% Usage:
%   run_triad_only;           % uses default_cfg(), then you edit the struct below
%   run_triad_only(cfg);      % pass fully specified cfg (preferred)
%
% No hidden defaults inside tasks: all inputs come from cfg.

% --- config ---------------------------------------------------------------
if nargin==0 || isempty(cfg)
    cfg = default_cfg();  % explicit starting values, visible here
    % Edit below if you want different files/method at runtime:
    cfg.dataset_id = 'X002';
    cfg.method     = 'TRIAD';
    cfg.imu_file   = 'IMU_X002.dat';
    cfg.gnss_file  = 'GNSS_X002.csv';
    cfg.truth_file = 'STATE_X001.txt';  % set '' if unavailable
end

% Paths (MATLAB and Python results are independent)
cfg.paths = project_paths();  % adds src/utils to path; ensures MATLAB/results exists

% Resolve absolute input paths
cfg.imu_path  = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path = fullfile(cfg.paths.root, cfg.gnss_file);
if ~isempty(cfg.truth_file)
    cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);
else
    cfg.truth_path = '';  % make explicit
end

% Sanity checks (no hidden defaults)
mustExist(cfg.imu_path,  'IMU file');
mustExist(cfg.gnss_path, 'GNSS file');

% Results dir (MATLAB-only)
results_dir = cfg.paths.matlab_results;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

fprintf('\u25b6 %s_%s_%s\n', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method);
fprintf('MATLAB results dir: %s\n', results_dir);

% --- Tasks 1..5 -----------------------------------------------------------
Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);  % will auto-run 1/2 if their .mat are missing
Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);

% --- Tasks 6..7 (optional truth) -----------------------------------------
task5_file = fullfile(results_dir, sprintf('%s_%s_%s_task5_results.mat', ...
                erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
if isfile(task5_file) && ~isempty(cfg.truth_path) && isfile(cfg.truth_path)
    disp('--- Running Task 6: Truth Overlay/Validation ---');
    Task_6(task5_file, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    run_id = sprintf('%s_%s_%s', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method);
    out_dir = fullfile(results_dir, run_id);
    fprintf('Task 6 overlay plots saved under: %s\n', out_dir);

    disp('--- Running Task 7: Residuals & Summary ---');
    Task_7();  % assumes it reads its own inputs from MATLAB/results
    fprintf('Task 7 evaluation plots saved under: %s\n', out_dir);
    disp('Task 6 and Task 7 complete. See MATLAB/results for plots and PDF summaries.');
else
    warning('Task 6/7 skipped: Missing Task 5 results or truth file (cfg.truth_path=%s).', cfg.truth_path);
end
end

% --- helpers --------------------------------------------------------------
function mustExist(p, label)
    if ~isfile(p)
        error('%s not found: %s', label, p);
    end
end
