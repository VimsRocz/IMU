clc; clear; close all;

% ensure output directory exists before running tasks
if ~exist('results','dir')
    mkdir('results');
end

fprintf('Running IMU+GNSS Initialization Pipeline (MATLAB Version)\n');

datasets = {
    'IMU_X001.dat','GNSS_X001.csv';
    'IMU_X002.dat','GNSS_X002.csv';
    'IMU_X003.dat','GNSS_X002.csv'};
for k = 1:size(datasets,1)
    [imu, gnss] = datasets{k,:};
    Task_1(imu, gnss);
    Task_2(imu, gnss);
    Task_3(imu, gnss);
    Task_4(imu, gnss);
    Task_5(imu, gnss);
end

fprintf('All tasks completed!\n');
