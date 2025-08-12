%% Test MATLAB Interactive Plotting with Synthetic Data
% This script tests the core MATLAB interactive plotting functionality

clear; clc; close all;

fprintf('Testing MATLAB Interactive Plotting Functions...\n');

% Check if we're in the right directory and add paths
if ~exist('MATLAB', 'dir')
    error('Please run this script from the repository root directory');
end

addpath('MATLAB');

% Create synthetic test data
fprintf('Generating synthetic test data...\n');
t = linspace(0, 100, 1000)';
n_points = length(t);

% Generate synthetic position, velocity, acceleration data
pos_est = [10*sin(0.1*t), 10*cos(0.1*t), 0.02*t];
vel_est = [cos(0.1*t), -sin(0.1*t), 0.02*ones(size(t))];
acc_est = [-0.1*sin(0.1*t), -0.1*cos(0.1*t), zeros(size(t))];

% Add noise for truth data
noise_scale = 0.1;
pos_truth = pos_est + noise_scale * randn(size(pos_est));
vel_truth = vel_est + noise_scale * 0.5 * randn(size(vel_est));
acc_truth = acc_est + noise_scale * 0.2 * randn(size(acc_est));

% Create output directory
output_dir = 'MATLAB/results/test_synthetic';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Test the core interactive plotting function directly
frames = {'ECEF', 'NED', 'Body'};
methods = {'TRIAD', 'Davenport', 'SVD'};

fprintf('\nTesting interactive 3x3 plotting function...\n');

for i = 1:length(methods)
    method = methods{i};
    for j = 1:length(frames)
        frame = frames{j};
        
        fprintf('  Creating plot for %s - %s frame...', method, frame);
        
        try
            % Call the interactive plotting function directly
            pdf_path = plot_overlay_interactive_3x3(t, pos_est, vel_est, acc_est, ...
                pos_truth, vel_truth, acc_truth, frame, method, ...
                'TEST_DATASET', output_dir);
            
            if exist(pdf_path, 'file')
                fprintf(' ✓ Success\n');
            else
                fprintf(' ✗ No output file\n');
            end
            
        catch ME
            fprintf(' ✗ Error: %s\n', ME.message);
            if strcmp(ME.identifier, 'MATLAB:UndefinedFunction')
                fprintf('     Function not found. Checking if file exists...\n');
                if exist('task6_overlay_plot_interactive.m', 'file')
                    fprintf('     File exists but function not accessible.\n');
                else
                    fprintf('     File not found in MATLAB path.\n');
                end
            end
        end
    end
end

% Test if core functions exist
fprintf('\nChecking function availability...\n');
functions_to_check = {
    'task6_overlay_plot_interactive', ...
    'plot_overlay_interactive_3x3', ...
    'get_results_dir'
};

for k = 1:length(functions_to_check)
    func_name = functions_to_check{k};
    if exist(func_name, 'file') == 2
        fprintf('  ✓ %s found\n', func_name);
    else
        fprintf('  ✗ %s not found\n', func_name);
    end
end

% List generated files
fprintf('\nGenerated files in %s:\n', output_dir);
if exist(output_dir, 'dir')
    files = dir(fullfile(output_dir, '*.*'));
    files = files(~[files.isdir]); % Remove directories
    
    if isempty(files)
        fprintf('  No files generated.\n');
    else
        for k = 1:length(files)
            file_size = files(k).bytes / (1024*1024); % Size in MB
            fprintf('  %s (%.1f MB)\n', files(k).name, file_size);
        end
    end
else
    fprintf('  Output directory not created.\n');
end

fprintf('\nTest completed!\n');