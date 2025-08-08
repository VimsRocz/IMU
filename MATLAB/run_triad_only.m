function run_triad_only(cfg)
% RUN_TRIAD_ONLY Execute Tasks 1..7 for a dataset using the TRIAD method.
% - MATLAB results go to <repo>/MATLAB/results (independent of Python's <repo>/results).
% - No hidden defaults inside tasks; everything comes from cfg.
% - Tasks 6 and 7 always run; if truth is missing, Task 6 falls back to
%   a GNSS-only overlay.

% --- ensure utils on path -------------------------------------------------
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src','utils'));
addpath(fullfile(here,'src'));
if exist(fullfile(here,'src','utils','attitude'),'dir')
    addpath(fullfile(here,'src','utils','attitude'));
end
if exist(fullfile(here,'src','utils','frames'),'dir')
    addpath(fullfile(here,'src','utils','frames'));
end

% --- config ---------------------------------------------------------------
if nargin==0 || isempty(cfg)
    cfg = default_cfg();              % visible starting values; tasks get all inputs from cfg
    cfg.dataset_id = 'X002';
    cfg.method     = 'TRIAD';
    cfg.imu_file   = 'IMU_X002.dat';
    cfg.gnss_file  = 'GNSS_X002.csv';
    % IMPORTANT: Task 6/7 are mandatory. Point to the correct truth file for your dataset.
    % If your repo uses a different naming, update this value accordingly.
    cfg.truth_file = '/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt';
end

% Resolve project paths and MATLAB-only results
cfg.paths = project_paths();                 % adds utils to path, returns root + matlab_results
mat_results = cfg.paths.matlab_results;      % <repo>/MATLAB/results

% Resolve absolute input paths
cfg.imu_path   = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path  = fullfile(cfg.paths.root, cfg.gnss_file);
cfg.truth_path = cfg.truth_file;

% Required inputs must exist (truth optional)
mustExist(cfg.imu_path,   'IMU file');
mustExist(cfg.gnss_path,  'GNSS file');
if ~isfile(cfg.truth_path)
    warning('Truth file not found: %s', cfg.truth_path);
end

run_id  = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);
results_dir = cfg.paths.matlab_results;
timeline_matlab(run_id, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
fprintf('> %s\n', run_id);
fprintf('MATLAB results dir: %s\n', results_dir);

% Expected outputs by task (for assertions)
t1_mat = fullfile(mat_results, sprintf('Task1_init_%s.mat', run_id));
t2_mat = fullfile(mat_results, sprintf('Task2_body_%s.mat', run_id));
t3_mat = fullfile(mat_results, sprintf('%s_task3_results.mat', run_id));
results_dir = get_results_dir();
t4_mat = fullfile(results_dir, sprintf('%s_task4_results.mat', run_id));
t5_mat = fullfile(mat_results, sprintf('%s_task5_results.mat', run_id));

% --- Run Tasks 1..5 ------------------------------------------------------
Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t1_mat, 'Task 1 results');

Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t2_mat, 'Task 2 results');

Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);   % consumes Task1/2 mats; will re-create if missing
assertFile(t3_mat, 'Task 3 results');

Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t4_mat, 'Task 4 results');

Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t5_mat, 'Task 5 results');

% --- Tasks 6..7 -----------------------------------------------------------
task5_file = fullfile(results_dir, sprintf('%s_task5_results.mat', run_id));
if ~isfile(task5_file)
    error('Task 6/7: Task 5 results not found: %s', task5_file);
end

Task_6(task5_file, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
Task_7();   % reads its own inputs under MATLAB/results

end % function

% --- helpers --------------------------------------------------------------
function mustExist(p, label)
if ~isfile(p)
    error('%s not found: %s', label, p);
end
end

function assertFile(p, label)
if ~isfile(p)
    error('%s was not created: %s', label, p);
else
    fprintf('   OK: %s\n', p);
end
end

