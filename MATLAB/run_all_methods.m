% Run all methods for Tasks 1–7
% Author: Auto-generated
% Date: 2025-08-15 09:48:55
% Description: Minimal orchestrator to run all methods and task scripts.

clear; clc;

% Resolve MATLAB directory for robust relative paths
this_file = mfilename('fullpath');
matlab_dir = fileparts(this_file);
orig_dir = pwd; cd(matlab_dir);
cleanupObj = onCleanup(@() cd(orig_dir));

% Environment
addpath(genpath(matlab_dir));
format long;
set(0, 'DefaultFigureVisible', 'on');

% Task 1–3: method-only runners
run('run_triad_only.m');
run('run_svd_only.m');
run('run_davenport_only.m');

% Task 4–7: main task scripts (if present)
task_dirs = {'Task_4','Task_5','Task_6','Task_7'};
task_scripts = {'Task_4.m','Task_5.m','Task_6.m','Task_7.m'};
for k = 1:numel(task_dirs)
    td = fullfile(matlab_dir, task_dirs{k});
    ts = fullfile(td, task_scripts{k});
    if exist(ts, 'file')
        cd(td);
        run(task_scripts{k});
        cd(matlab_dir);
    else
        fprintf('Skipping %s (missing %s)\n', task_dirs{k}, task_scripts{k});
    end
end
