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
Task_5(imu_path, gnss_path, method);

% ------------------------------------------------------------------
% Optional Tasks 6 and 7 when Task 5 output exists
% ------------------------------------------------------------------
task5_file = fullfile(results_dir, sprintf('IMU_X002_GNSS_X002_%s_task5_results.mat', method));
if isfile(task5_file)
    S = load(task5_file);
    if isfield(S,'pos_ned') && isfield(S,'gnss_pos_ned')
        Task_6(task5_file, imu_path, gnss_path, 'STATE_X001.txt');
        Task_7(S.pos_ned, S.gnss_pos_ned, method);
    end
end

%% Load and display rotation matrix from Task 3 results
task3_file = fullfile(results_dir, 'Task3_results_IMU_X002_GNSS_X002.mat');
if exist(task3_file, 'file')
    data = load(task3_file);
    if isfield(data, 'task3_results') && isfield(data.task3_results, 'TRIAD')
        C_B_N = data.task3_results.TRIAD.R;
        fprintf('\nRotation matrix C_{B}^{N}:\n');
        disp(C_B_N);
    end
end
fprintf('Results saved to %s\n', results_dir);
