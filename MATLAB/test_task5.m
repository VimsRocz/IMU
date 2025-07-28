% Test script to check Task 5 with minimal prerequisite data
addpath('.');

% Ensure Task 1 is complete
if ~exist('results/Task1_IMU_X002_GNSS_X002_TRIAD.mat', 'file')
    Task_1('/home/runner/work/IMU/IMU/IMU_X002.dat', '/home/runner/work/IMU/IMU/GNSS_X002.csv', 'TRIAD', 'IMU_X002_GNSS_X002');
end

% Create minimal Task 2 output
task2_file = 'results/Task2_IMU_X002_GNSS_X002_TRIAD.mat';
if ~exist(task2_file, 'file')
    fprintf('Creating minimal Task 2 data...\n');
    accel_bias = [0.577573, -6.836713, 0.910290];
    gyro_bias = [-2.322999e-05, -2.802118e-06, 6.900074e-05];
    accel_scale = 1.0;
    save(task2_file, 'accel_bias', 'gyro_bias', 'accel_scale');
    fprintf('Saved minimal Task 2 data\n');
end

% Create minimal Task 3 output
task3_file = 'results/Task3_IMU_X002_GNSS_X002_TRIAD.mat';
if ~exist(task3_file, 'file')
    fprintf('Creating minimal Task 3 data...\n');
    % From Python output: TRIAD rotation matrix
    R_triad = [2.33646980e-01, -4.54035202e-02, 9.71260835e-01;
               1.06220955e-02,  9.98968728e-01, 4.41435243e-02;
              -9.72263472e-01,  2.82418914e-06, 2.33888307e-01];
    task3_results.TRIAD.R = R_triad;
    save(task3_file, 'task3_results');
    fprintf('Saved minimal Task 3 data\n');
end

% Create minimal Task 4 output
task4_file = 'results/Task4_IMU_X002_GNSS_X002_TRIAD.mat';
if ~exist(task4_file, 'file')
    fprintf('Creating minimal Task 4 data...\n');
    % Create dummy position, velocity, acceleration data
    n_samples = 100;  % Start with small data for testing
    pos_ned = zeros(3, n_samples);
    vel_ned = zeros(3, n_samples);
    acc_ned = zeros(3, n_samples-1);
    save(task4_file, 'pos_ned', 'vel_ned', 'acc_ned');
    fprintf('Saved minimal Task 4 data\n');
end

% Now try Task 5
fprintf('Attempting to run Task 5...\n');
try
    Task_5('/home/runner/work/IMU/IMU/IMU_X002.dat', '/home/runner/work/IMU/IMU/GNSS_X002.csv', 'TRIAD', 'IMU_X002_GNSS_X002');
    fprintf('Task 5 completed successfully!\n');
catch e
    fprintf('Task 5 failed: %s\n', e.message);
    for i = 1:length(e.stack)
        fprintf('  at %s line %d\n', e.stack(i).name, e.stack(i).line);
    end
end