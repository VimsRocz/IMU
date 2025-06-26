clc; clear; close all;

% ensure output directory exists before running tasks
if ~exist('results','dir')
    mkdir('results');
end

fprintf('Running IMU+GNSS Initialization Pipeline (MATLAB Version)\n');
imuFile = get_data_file('IMU_X001.dat');
gnssFile = get_data_file('GNSS_X001.csv');
Task_1(imuFile, gnssFile);
Task_2();
Task_3();
Task_4();
Task_5();

fprintf('All tasks completed!\n');
