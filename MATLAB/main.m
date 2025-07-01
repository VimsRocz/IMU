function main(imu_path, gnss_path, methods)
%MAIN Run the IMU+GNSS initialization pipeline on a single dataset or list.
%   main()                              - use all sample files with all methods
%   main('imu.dat','gnss.csv')          - run one dataset with all methods
%   main('imu.dat','gnss.csv','TRIAD')  - run one dataset with a single method
%   main({'IMU_X001.dat','IMU_X002.dat'}, {'GNSS_X001.csv','GNSS_X002.csv'}, ...
%        {'TRIAD','SVD'})               - iterate over both dataset pairs and
%                                         selected methods

% Resolve default data file paths or lists. The pipeline ships with three IMU
% logs and two GNSS logs. By default we pair them as:
%   IMU_X001.dat <-> GNSS_X001.csv
%   IMU_X002.dat <-> GNSS_X002.csv
%   IMU_X003.dat <-> GNSS_X002.csv
imu_list = {
    get_data_file('IMU_X001.dat'), ...
    get_data_file('IMU_X002.dat'), ...
    get_data_file('IMU_X003.dat')  ...
    };
gnss_list = {
    get_data_file('GNSS_X001.csv'), ...
    get_data_file('GNSS_X002.csv'), ...
    get_data_file('GNSS_X002.csv')  ...
    };

% If the user passes in a single string, use it as a single item list
if nargin >= 1 && ~isempty(imu_path)
    if ischar(imu_path) || isstring(imu_path)
        imu_list = {char(imu_path)};
    else
        imu_list = cellfun(@char, imu_path, 'UniformOutput', false);
    end
end
if nargin >= 2 && ~isempty(gnss_path)
    if ischar(gnss_path) || isstring(gnss_path)
        gnss_list = {char(gnss_path)};
    else
        gnss_list = cellfun(@char, gnss_path, 'UniformOutput', false);
    end
end

% Optional third argument: list of methods to run
method_list = {'TRIAD','Davenport','SVD'};
if nargin >= 3 && ~isempty(methods)
    if ischar(methods) || isstring(methods)
        method_list = {char(methods)};
    else
        method_list = cellfun(@char, methods, 'UniformOutput', false);
    end
end

if numel(imu_list) ~= numel(gnss_list)
    error('IMU and GNSS file lists must have the same length');
end

clc; close all;

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir,'dir'); mkdir(results_dir); end

fprintf('Running IMU+GNSS Initialization Pipeline (MATLAB Version)\n');

methods = method_list;

for dataIdx = 1:numel(imu_list)
    imu_file  = imu_list{dataIdx};
    gnss_file = gnss_list{dataIdx};

    fprintf('\n=== Running pipeline for dataset: %s + %s ===\n', imu_file, gnss_file);

    for mIdx = 1:numel(methods)
        method = methods{mIdx};
        fprintf('\n=== Running pipeline for method: %s ===\n', method);
        try
            t1 = Task_1(imu_file, gnss_file, method);
            t2 = Task_2(imu_file, gnss_file, method);
            t3 = Task_3(imu_file, gnss_file, method);
            t4 = Task_4(imu_file, gnss_file, method);
            t5 = Task_5(imu_file, gnss_file, method);
            assignin('base','task1_results',t1);
            assignin('base','task2_results',t2);
            assignin('base','task3_results',t3);
            assignin('base','task4_results',t4);
            assignin('base','task5_results',t5);
        catch ME
            fprintf('Error in method %s: %s\n', method, ME.message);
            continue;
        end
        fprintf('Finished dataset %s + %s with method %s\n', imu_file, gnss_file, method);
    end
end

fprintf('\nAll datasets and methods completed!\n');
end
