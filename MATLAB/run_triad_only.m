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
    cfg.truth_file = 'STATE_X001.txt';
end

% Resolve project paths and MATLAB-only results
cfg.paths = project_paths();                 % adds src/utils to path, returns root + matlab_results
mat_results = cfg.paths.matlab_results;      % <repo>/MATLAB/results
if ~exist(mat_results,'dir'), mkdir(mat_results); end

% Resolve absolute input paths
cfg.imu_path   = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path  = fullfile(cfg.paths.root, cfg.gnss_file);
cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);

% Required inputs must exist (truth optional)
mustExist(cfg.imu_path,   'IMU file');
mustExist(cfg.gnss_path,  'GNSS file');
if ~isfile(cfg.truth_path)
    warning('Truth file not found: %s', cfg.truth_path);
end

imu_id  = erase(cfg.imu_file,  {'.dat','.DAT'});
gnss_id = erase(cfg.gnss_file, {'.csv','.CSV'});
run_id  = sprintf('%s_%s_%s', imu_id, gnss_id, cfg.method);
results_dir = cfg.paths.matlab_results;
timeline_txt = fullfile(results_dir, [run_id '_timeline.txt']);
timeline_summary(run_id, cfg.imu_path, cfg.gnss_path, cfg.truth_path, timeline_txt);
fprintf('> %s\n', run_id);
fprintf('MATLAB results dir: %s\n', results_dir);

% Expected outputs by task (for assertions)
t1_mat = fullfile(mat_results, sprintf('Task1_init_%s_%s_%s.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t2_mat = fullfile(mat_results, sprintf('Task2_body_%s_%s_%s.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t3_mat = fullfile(mat_results, sprintf('%s_%s_%s_task3_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
results_dir = get_results_dir();
[~, imu_base, ~]  = fileparts(cfg.imu_path);
[~, gnss_base, ~] = fileparts(cfg.gnss_path);
imu_base  = erase(imu_base,  '.dat');
gnss_base = erase(gnss_base, '.csv');
run_id = sprintf('%s_%s_%s', imu_base, gnss_base, cfg.method);
t4_mat = fullfile(results_dir, sprintf('%s_task4_results.mat', run_id));
t5_mat = fullfile(mat_results, sprintf('%s_%s_%s_task5_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));

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
task5_file = fullfile(results_dir, sprintf('%s_%s_%s_task5_results.mat', ...
                   erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
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

