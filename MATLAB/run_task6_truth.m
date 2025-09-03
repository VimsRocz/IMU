function run_task6_truth(task5_file, truth_file)
%RUN_TASK6_TRUTH Run Task_6 with optional truth overlay and exit.
%   run_task6_truth(task5_file, truth_file) calls Task_6 with empty IMU and
%   GNSS paths. TRUTH_FILE may be empty; in that case Task_6 will attempt to
%   locate the reference via resolve_truth_path. This helper is intended for
%   non-interactive CLI use, e.g.:
%
%   matlab -batch "addpath('MATLAB'); run_task6_truth('results/task5.mat','STATE_X001.txt')"
%
%   On completion MATLAB exits automatically. A non-zero exit status is
%   used when Task_6 throws an error.
%
% Inputs:
%   task5_file  - Path to Task 5 result file.
%   truth_file  - Path to truth state file (optional).
%
% See also: Task_6, resolve_truth_path

if nargin < 2
    truth_file = '';
end

try
    Task_6(task5_file, '', '', truth_file);
    exit;
catch ME
    fprintf('run_task6_truth: %s\n', ME.message);
    exit(1);
end
end
