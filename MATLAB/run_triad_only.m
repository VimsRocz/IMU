%RUN_TRIAD_ONLY  Process dataset X002 using the TRIAD method.
%   This wrapper mirrors ``src/run_triad_only.py``. It resolves the
%   dataset files via ``check_files`` and forwards them to
%   ``GNSS_IMU_Fusion_single`` which performs Tasks 1--7 including
%   static-interval detection and Kalman filtering. All results are moved
%   to ``results/IMU_X002_GNSS_X002_TRIAD`` so the output layout matches
%   the Python helper.
%
%   Usage:
%       run_triad_only
%
%   The printed rotation matrix should match the Python value
%   [0.2336 -0.0454 0.9713; 0.0106 0.9990 0.0441; -0.9723 0.0000 0.2339]
%   up to numerical precision.

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
% Validate that the dataset files exist before invoking the pipeline. The
% return values are discarded so ``GNSS_IMU_Fusion_single`` resolves the
% paths itself via ``get_data_file`` just like the Python helper. Passing
% absolute paths here would fail because ``GNSS_IMU_Fusion_single`` expects
% simple filenames.
check_files(imu_file, gnss_file);


out_dir = fullfile(get_results_dir(), 'IMU_X002_GNSS_X002_TRIAD');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% ``GNSS_IMU_Fusion_single`` internally looks up ``imu_file`` and
% ``gnss_file`` using ``get_data_file`` so only the filenames are passed
% here. This keeps the behaviour consistent with ``src/run_triad_only.py``.
GNSS_IMU_Fusion_single(imu_file, gnss_file, 'TRIAD');

%% Move generated files into the dedicated folder
results_dir = get_results_dir();
tag = 'IMU_X002_GNSS_X002_TRIAD';
files = dir(fullfile(results_dir, [tag '*']));
for k = 1:numel(files)
    movefile(fullfile(results_dir, files(k).name), fullfile(out_dir, files(k).name), 'f');
end

%% Load and display rotation matrix from Task 3 results
task3_file = fullfile(out_dir, 'Task3_results_IMU_X002_GNSS_X002.mat');
if exist(task3_file, 'file')
    data = load(task3_file);
    if isfield(data, 'task3_results') && isfield(data.task3_results, 'TRIAD')
        C_B_N = data.task3_results.TRIAD.R;
        fprintf('\nRotation matrix C_{B}^{N}:\n');
        disp(C_B_N);
    end
end
fprintf('Results saved to %s\n', out_dir);

%% -------------------------------------------------------------------------
function [imu_path, gnss_path] = check_files(imu_file, gnss_file)
%CHECK_FILES  Return validated paths for IMU and GNSS data files.
%   The helper searches for each file in the following locations:
%     1. ``Data/`` under the repository root
%     2. Repository root
%     3. Current working directory

    script_dir = fileparts(mfilename('fullpath'));
    repo_root  = fileparts(script_dir);

    search_dirs = {fullfile(repo_root, 'Data'), repo_root, pwd};

    imu_path = '';
    gnss_path = '';
    for i = 1:numel(search_dirs)
        cand = fullfile(search_dirs{i}, imu_file);
        if isempty(imu_path) && exist(cand, 'file')
            imu_path = cand; end
        cand = fullfile(search_dirs{i}, gnss_file);
        if isempty(gnss_path) && exist(cand, 'file')
            gnss_path = cand; end
    end

    if isempty(imu_path)
        error('File not found: %s', imu_file);
    end
    if isempty(gnss_path)
        error('File not found: %s', gnss_file);
    end
end
