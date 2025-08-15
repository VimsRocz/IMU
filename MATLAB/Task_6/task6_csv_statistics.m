function [mean_val, median_val, mode_val] = task6_csv_statistics(csv_file, output_dir)
%TASK6_CSV_STATISTICS  Basic CSV statistics and visualization.
%
%   [MEAN_VAL, MEDIAN_VAL, MODE_VAL] = TASK6_CSV_STATISTICS(CSV_FILE, OUTPUT_DIR)
%   reads ``CSV_FILE`` (default ``'data_task6.csv'``), computes the mean,
%   median and mode of its first numeric column and creates a histogram.
%   The figure is saved as ``task6_histogram.pdf`` and ``.png`` under
%   ``OUTPUT_DIR`` (default ``get_results_dir()``). Debugging information is
%   printed during execution.
%
%   Usage:
%       stats = task6_csv_statistics();
%
%   See also TASK6_CSV_STATISTICS.PY, READTABLE, HISTOGRAM.

if nargin < 1 || isempty(csv_file)
    csv_file = 'data_task6.csv';
end
if nargin < 2 || isempty(output_dir)
    output_dir = get_results_dir();
end

% Ensure output directory exists
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('Starting Task 6: Reading and visualizing data from %s\n', csv_file);

% ------------------------------------------------------------------
% Read the CSV file with basic error handling
try
    fprintf('Attempting to read ''%s'' ...\n', csv_file);
    data = readtable(csv_file);
    fprintf('File read successfully. Checking data contents ...\n');
catch ME
    fprintf('Error: Failed to read %s (%s)\n', csv_file, ME.message);
    mean_val = [];
    median_val = [];
    mode_val = [];
    return;
end

% ------------------------------------------------------------------
% Validate first column is numeric
if isempty(data)
    fprintf('Error: CSV appears empty.\n');
    mean_val = [];
    median_val = [];
    mode_val = [];
    return;
end

first_col = data{:,1};
if ~isnumeric(first_col)
    fprintf('Error: The file contains invalid non-numerical data.\n');
    mean_val = [];
    median_val = [];
    mode_val = [];
    return;
end

fprintf('Data validation passed. Working with column: %s\n', data.Properties.VariableNames{1});

fprintf('First few rows of data:\n');
disp(data(1:min(5,height(data)), :));

fprintf('Extracted numerical data (first 10 values):\n');
disp(first_col(1:min(10, numel(first_col)))');

% ------------------------------------------------------------------
% Calculate statistics
try
    mean_val = mean(first_col);
    median_val = median(first_col);
    mode_val = mode(first_col);
    fprintf('Statistical calculations completed successfully.\n');
catch ME
    fprintf('Error: Failed to calculate statistics (%s)\n', ME.message);
    mean_val = [];
    median_val = [];
    mode_val = [];
    return;
end

fprintf('Mean of the data: %g\n', mean_val);
fprintf('Median of the data: %g\n', median_val);
fprintf('Mode of the data: %g\n', mode_val);

% ------------------------------------------------------------------
% Create histogram and save as PDF and PNG
fprintf('Generating histogram ...\n');

fig = figure('Visible', 'off');
histogram(first_col, 10, 'FaceColor', [0.5 0.7 1], 'EdgeColor', 'black');
title(sprintf('Histogram of %s', data.Properties.VariableNames{1}));
xlabel('Value');
ylabel('Frequency');
grid on;

base = fullfile(output_dir, 'task6_histogram');
savefig(gcf); % replaced for interactive
savefig(fig);
close(fig);

fprintf('Histogram saved as %s.[pdf,png]\n', base);

fprintf('Returning statistical measures: [mean, median, mode]\n');
end
