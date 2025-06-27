function main(imuFile, gnssFile)
%MAIN Run the IMU+GNSS initialization pipeline on a single dataset.
%   main()                      - use the default sample files
%   main('imu.dat','gnss.csv') - run with custom data files

if nargin < 1 || isempty(imuFile)
    imuFile = 'IMU_X001.dat';
end
if nargin < 2 || isempty(gnssFile)
    gnssFile = 'GNSS_X001.csv';
end

clc; close all;

% ensure output directory exists before running tasks
if ~exist('results','dir')
    mkdir('results');
end

fprintf('Running IMU+GNSS Initialization Pipeline (MATLAB Version)\n');

Task_1(imuFile, gnssFile);
Task_2(imuFile, gnssFile);

methods = {'TRIAD','Davenport','SVD'};
for i = 1:numel(methods)
    method = methods{i};
    Task_3(imuFile, gnssFile, method);
    Task_4(imuFile, gnssFile, method);
end

results_dir = 'results';
tmp = load(fullfile(results_dir,'task4_results.mat'), 'gnss_pos_ned');
for m = methods
    Task_5(imuFile, gnssFile, m{1}, tmp.gnss_pos_ned);
end

fprintf('All tasks completed!\n');
end
