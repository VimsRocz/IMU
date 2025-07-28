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

results_dir = get_results_dir();
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

% ------------------------------------------------------------------
% Execute Tasks 1--5 sequentially using the TRIAD initialisation
% ------------------------------------------------------------------
Task_1(imu_path, gnss_path, method);
Task_2(imu_path, gnss_path, method);
Task_3(imu_path, gnss_path, method);
Task_4(imu_path, gnss_path, method);
% Task_5 accepts the file paths for compatibility but ignores them in the
% current simplified implementation.
Task_5(imu_path, gnss_path, method);
% Demonstration: run simplified Kalman filter loop and save ``x_log`` only
% This mirrors the Python stub ``task5_kf_state_log.py`` and shows how to
% persist the state history matrix to ``MATLAB/results``.
task5_kf_state_log();

% ------------------------------------------------------------------
% Tasks 6 and 7: validation and residual analysis
% ------------------------------------------------------------------
task5_file = fullfile(results_dir, sprintf('IMU_X002_GNSS_X002_%s_task5_results.mat', method));
truth_file = fullfile(root_dir, 'STATE_X001.txt');
if isfile(task5_file) && isfile(truth_file)
    disp('--- Running Task 6: Truth Overlay/Validation ---');
    % Task_6 currently uses internally defined file paths and therefore
    % accepts no input arguments. Call it without parameters to avoid the
    % "Too many input arguments" error.
    Task_6();
    [~, imu_name, ~]  = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
    run_id = sprintf('%s_%s_%s', imu_name, gnss_name, method);
    out_dir = fullfile(results_dir, run_id);
    fprintf('Task 6 overlay plots saved under: %s\n', out_dir);
    disp('--- Running Task 7: Residuals & Summary ---');
    Task_7(task5_file, truth_file, run_id);
    fprintf('Task 7 evaluation plots saved under: %s\n', out_dir);
    disp('Task 6 and Task 7 complete. See results directory for plots and PDF summaries.');
else
    warning('Task 6 or Task 7 skipped: Missing Task 5 results or truth file.');
end
