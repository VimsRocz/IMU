%% Test MATLAB Interactive Task 6 Plotting
% This script tests the MATLAB interactive plotting functionality
% using existing pipeline results

clear; clc; close all;

fprintf('Testing MATLAB Interactive Task 6 Plotting...\n');

% Check if we're in the right directory
if ~exist('MATLAB', 'dir')
    error('Please run this script from the repository root directory');
end

% Add MATLAB paths
addpath('MATLAB');
addpath('MATLAB/src');

% Look for existing result files
results_dir = 'PYTHON/results';
mat_files = dir(fullfile(results_dir, '*_kf_output.mat'));

if isempty(mat_files)
    error('No Kalman filter output files found. Run the Python pipeline first.');
end

% Use the first available file
kf_file = fullfile(results_dir, mat_files(1).name);
fprintf('Using result file: %s\n', kf_file);

% Look for truth file
truth_file = 'DATA/Truth/STATE_X001.txt';
if ~exist(truth_file, 'file')
    error('Truth file not found: %s', truth_file);
end

% Set output directory
output_dir = 'MATLAB/results/test_interactive';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Test different frames and methods
frames = {'ECEF', 'NED', 'Body'};
methods = {'TRIAD', 'Davenport', 'SVD'};

fprintf('\nTesting interactive plots for different configurations...\n');

for i = 1:length(methods)
    method = methods{i};
    for j = 1:length(frames)
        frame = frames{j};
        
        fprintf('  Creating interactive plot for %s - %s frame...', method, frame);
        
        try
            % Test the interactive plotting function
            pdf_path = task6_overlay_plot(kf_file, truth_file, method, frame, ...
                                        'TEST_DATASET', output_dir, false, true);
            
            if exist(pdf_path, 'file')
                fprintf(' ✓ Success\n');
            else
                fprintf(' ✗ Failed (no output file)\n');
            end
            
        catch ME
            fprintf(' ✗ Failed: %s\n', ME.message);
        end
    end
end

% List generated files
fprintf('\nGenerated files in %s:\n', output_dir);
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

fprintf('\nMATLAB Interactive Features:\n');
fprintf('  • Zoom: Use zoom tool or mouse scroll wheel\n');
fprintf('  • Pan: Use pan tool or click and drag\n');
fprintf('  • Data cursor: Click on data points for exact values\n');
fprintf('  • Reset view: Double-click on subplot\n');
fprintf('  • Keyboard shortcuts: F (fullscreen), G (grid toggle), R (reset)\n');
fprintf('  • Custom toolbar: Reset all views, toggle grid\n');

fprintf('\nTest completed!\n');