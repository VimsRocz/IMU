%RUN_TRIAD_ONLY  Process dataset X002 using the TRIAD method.
%   This wrapper mirrors ``src/run_triad_only.py``. Dataset files are
%   referenced directly in the repository root and forwarded to
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
root_dir  = fileparts(fileparts(mfilename('fullpath')));
imu_path  = fullfile(root_dir, imu_file);
gnss_path = fullfile(root_dir, gnss_file);


out_dir = fullfile(get_results_dir(), 'IMU_X002_GNSS_X002_TRIAD');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

GNSS_IMU_Fusion_single(imu_path, gnss_path, 'TRIAD');

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
