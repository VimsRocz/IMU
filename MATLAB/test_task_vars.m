% Quick test to run Tasks 1-4 and check what variables are available for Task 5
addpath('.');

% Run Tasks 1-4
Task_1('/home/runner/work/IMU/IMU/IMU_X002.dat', '/home/runner/work/IMU/IMU/GNSS_X002.csv', 'TRIAD', 'IMU_X002_GNSS_X002');
fprintf('Task 1 completed\n');

% Check Task 1 outputs
task1_file = 'results/Task1_IMU_X002_GNSS_X002_TRIAD.mat';
if exist(task1_file, 'file')
    vars1 = load(task1_file);
    fprintf('Task 1 variables: %s\n', strjoin(fieldnames(vars1), ', '));
else
    fprintf('Task 1 file not found\n');
end

% Load more carefully to check specific variables needed by Task 5
try
    load(task1_file, 'gravity_ned', 'lat0_rad', 'lon0_rad');
    fprintf('Task 1 variables for Task 5 loaded successfully\n');
    fprintf('  gravity_ned: %s\n', mat2str(gravity_ned));
    fprintf('  lat0_rad: %f\n', lat0_rad);
    fprintf('  lon0_rad: %f\n', lon0_rad);
catch e
    fprintf('Failed to load Task 1 variables for Task 5: %s\n', e.message);
end