%RUN_TRIAD_ONLY  Process dataset X002 using the TRIAD method.
%   This script mirrors ``src/run_triad_only.py`` but follows the
%   ``run_all_methods.m`` approach where Tasks 1--7 are executed
%   sequentially in MATLAB. Dataset files are referenced directly from the
%   repository root and all outputs are written to ``results/`` using the
%   standard naming convention. The printed rotation matrix should match
%   the Python value
%   ``[0.2336 -0.0454 0.9713; 0.0106 0.9990 0.0441; -0.9723 0.0000 0.2339]``
%   up to numerical precision.
%
%   Usage:
%       run_triad_only


%% Resolve helper path
st = dbstack('-completenames');
if ~isempty(st)
    script_dir = fileparts(st(1).file);
else
    script_dir = fileparts(mfilename('fullpath'));
end
addpath(script_dir);

imu_file  = 'IMU_X002.dat';
gnss_file = 'GNSS_X002.csv';
method    = 'TRIAD';

root_dir  = fileparts(fileparts(mfilename('fullpath')));
imu_path  = fullfile(root_dir, imu_file);
gnss_path = fullfile(root_dir, gnss_file);
[~, imu_name, ~]  = fileparts(imu_file);
[~, gnss_name, ~] = fileparts(gnss_file);
dataset_tag = sprintf('%s_%s', imu_name, gnss_name);
run_id = sprintf('%s_%s', dataset_tag, method); % used by Tasks 6 and 7

results_dir = get_results_dir();
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

% Mirror the initial console output from ``run_triad_only.py``
fprintf('▶ %s_%s_%s\n', imu_name, gnss_name, method);
fprintf('Ensured ''%s'' directory exists.\n', results_dir);
start_t = tic; % measure runtime similar to the Python helper

% ------------------------------------------------------------------
% Execute Tasks 1--5 sequentially using the TRIAD initialisation
% ------------------------------------------------------------------
Task_1(imu_path, gnss_path, method, dataset_tag);
Task_2(imu_path, gnss_path, method, dataset_tag);
Task_3(imu_path, gnss_path, method, dataset_tag);
Task_4(imu_path, gnss_path, method, dataset_tag);
% Task_5 accepts the file paths for compatibility but ignores them in the
% current simplified implementation.
Task_5(imu_path, gnss_path, method, dataset_tag);
% Demonstration: run simplified Kalman filter loop and save ``x_log`` only
% This mirrors the Python stub ``task5_kf_state_log.py`` and shows how to
% persist the state history matrix to ``MATLAB/results``.
task5_kf_state_log();

% ------------------------------------------------------------------
% Tasks 6 and 7: validation and residual analysis
% ------------------------------------------------------------------
task5_file = fullfile(results_dir, sprintf('Task5_%s_%s.mat', dataset_tag, method));
truth_file = fullfile(root_dir, 'STATE_X001.txt');
if isfile(task5_file) && isfile(truth_file)
    disp('--- Running Task 6: Truth Overlay/Validation ---');

    out_dir = fullfile(results_dir, run_id);
    fprintf('Task 6 overlay plots saved under: %s\n', out_dir);
    disp('--- Running Task 7: Residuals & Summary ---');
    Task_7(task5_file, truth_file, run_id);
    fprintf('Task 7 evaluation plots saved under: %s\n', out_dir);
    disp('Task 6 and Task 7 complete. See results directory for plots and PDF summaries.');
else
    warning('Task 6 or Task 7 skipped: Missing Task 5 results or truth file.');
end

elapsed_s = toc(start_t);
fprintf('Runtime %.2f s\n', elapsed_s);
disp('TRIAD processing complete for X002');
