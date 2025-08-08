function run_triad_only(cfg)
% RUN_TRIAD_ONLY — Execute Tasks 1..7 for a dataset using the TRIAD method
% - MATLAB results go to <repo>/MATLAB/results (independent of Python's <repo>/results).
% - No hidden defaults inside tasks; everything comes from cfg.
% - Task 6 and 7 are MANDATORY (like Python). If truth is missing, we error out.

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

% Required inputs must exist (Task 6/7 mandatory like Python)
mustExist(cfg.imu_path,   'IMU file');
mustExist(cfg.gnss_path,  'GNSS file');
mustExist(cfg.truth_path, 'Truth file (required for Task 6/7)');

imu_id  = erase(cfg.imu_file,  {'.dat','.DAT'});
gnss_id = erase(cfg.gnss_file, {'.csv','.CSV'});
run_id  = sprintf('%s_%s_%s', imu_id, gnss_id, cfg.method);
fprintf('%s %s_%s_%s\n', char(9654), imu_id, gnss_id, cfg.method);
fprintf('MATLAB results dir: %s\n', cfg.paths.matlab_results);
timeline_summary(run_id, cfg.imu_path, cfg.gnss_path, cfg.truth_path, cfg.paths.matlab_results);

% Expected outputs by task (for assertions)
t1_mat = fullfile(mat_results, sprintf('Task1_init_%s_%s_%s.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t2_mat = fullfile(mat_results, sprintf('Task2_body_%s_%s_%s.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t3_mat = fullfile(mat_results, sprintf('%s_%s_%s_task3_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t4_mat = fullfile(mat_results, sprintf('IMU_%s_GNSS_%s_%s_task4_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
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

% --- Tasks 6..7 (mandatory, like Python) ---------------------------------
p = project_paths();
results_dir = p.matlab_results;

task5_file = fullfile(results_dir, sprintf('%s_%s_%s_task5_results.mat', ...
                   erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
if ~isfile(task5_file)
    error('Task 6/7: Task 5 results not found: %s', task5_file);
end

if isempty(cfg.truth_file)
    error('Task 6/7: cfg.truth_file not set (truth is mandatory).');
end
cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);
if ~isfile(cfg.truth_path)
    error('Task 6/7: truth file not found: %s', cfg.truth_path);
end

disp('--- Running Task 6: Truth Overlay/Validation ---');
Task_6(task5_file, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
run_id = sprintf('%s_%s_%s', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method);
out_dir = fullfile(results_dir, run_id);
fprintf('Task 6 overlay plots saved under: %s\n', out_dir);

disp('--- Running Task 7: Residuals & Summary ---');
Task_7();  % reads its inputs from MATLAB/results
fprintf('Task 7 evaluation plots saved under: %s\n', out_dir);
disp('Task 6 and Task 7 complete. See MATLAB/results for plots and PDF summaries.');

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

