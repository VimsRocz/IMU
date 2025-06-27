function main(imu_path, gnss_path)
%MAIN Run the IMU+GNSS initialization pipeline on a single dataset.
%   main()                      - use the default sample files
%   main('imu.dat','gnss.csv') - run with custom data files

% Resolve default data file paths
imu_file  = get_data_file('IMU_X001.dat');
gnss_file = get_data_file('GNSS_X001.csv');

if nargin >= 1 && ~isempty(imu_path)
    imu_file = imu_path;
end
if nargin >= 2 && ~isempty(gnss_path)
    gnss_file = gnss_path;
end

clc; close all;

% ensure output directory exists before running tasks
if ~exist('results','dir')
    mkdir('results');
end

fprintf('Running IMU+GNSS Initialization Pipeline (MATLAB Version)\n');

Task_1(imu_file, gnss_file);
Task_2(imu_file, gnss_file);

methods = {'TRIAD','Davenport','SVD'};
for i = 1:numel(methods)
    method = methods{i};

end

fprintf('All tasks completed!\n');
end
