clc; clear; close all;

% ensure output directory exists before running tasks
if ~exist('results','dir')
    mkdir('results');
end

fprintf('Running IMU+GNSS Initialization Pipeline (MATLAB Version)\n');

Task_1();
Task_2();
Task_3();
Task_4();
Task_5();

fprintf('All tasks completed!\n');
