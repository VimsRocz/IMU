% Complete pipeline test with proper data flow
addpath('.');

% Clean start
fprintf('=== Testing Complete MATLAB Pipeline ===\n');

% Remove old results to start fresh
if exist('results', 'dir')
    rmdir('results', 's');
end
mkdir('results');

% Define paths
imu_path = '/home/runner/work/IMU/IMU/IMU_X002.dat';
gnss_path = '/home/runner/work/IMU/IMU/GNSS_X002.csv';
method = 'TRIAD';
dataset_tag = 'IMU_X002_GNSS_X002';

% Run pipeline
fprintf('\n--- Task 1: Reference vectors ---\n');
try
    Task_1(imu_path, gnss_path, method, dataset_tag);
    fprintf('✓ Task 1 completed\n');
catch e
    fprintf('✗ Task 1 failed: %s\n', e.message);
    return;
end

fprintf('\n--- Task 2: Body vectors ---\n');
try
    % This will take a while with the full dataset, but we need real biases
    Task_2(imu_path, gnss_path, method, dataset_tag);
    fprintf('✓ Task 2 completed\n');
catch e
    fprintf('✗ Task 2 failed: %s\n', e.message);
    return;
end

fprintf('\n--- Task 3: Attitude determination ---\n');
try
    Task_3(imu_path, gnss_path, method, dataset_tag);
    fprintf('✓ Task 3 completed\n');
catch e
    fprintf('✗ Task 3 failed: %s\n', e.message);
    return;
end

fprintf('\n--- Task 4: IMU integration ---\n');
try
    Task_4(imu_path, gnss_path, method, dataset_tag);
    fprintf('✓ Task 4 completed\n');
catch e
    fprintf('✗ Task 4 failed: %s\n', e.message);
    return;
end

fprintf('\n--- Task 5: Kalman filtering ---\n');
try
    Task_5(imu_path, gnss_path, method, dataset_tag);
    fprintf('✓ Task 5 completed\n');
catch e
    fprintf('✗ Task 5 failed: %s\n', e.message);
    return;
end

fprintf('\n=== Pipeline completed successfully! ===\n');

% Show final results
fprintf('\nFinal Task 5 variables:\n');
load('results/Task5_IMU_X002_GNSS_X002_TRIAD.mat');
fprintf('  pos_fused_ned: %s\n', mat2str(size(pos_fused_ned)));
fprintf('  vel_fused_ned: %s\n', mat2str(size(vel_fused_ned)));
fprintf('  x_log: %s\n', mat2str(size(x_log)));
fprintf('First position NED: [%.4f, %.4f, %.4f]\n', pos_fused_ned(1,1), pos_fused_ned(2,1), pos_fused_ned(3,1));
fprintf('Last position NED: [%.4f, %.4f, %.4f]\n', pos_fused_ned(1,end), pos_fused_ned(2,end), pos_fused_ned(3,end));