function result = TRIAD(imuFile, gnssFile)
%TRIAD Run the pipeline using only the TRIAD method.
%   RESULT = TRIAD(IMUFILE, GNSSFILE) executes Tasks 1--5 with the TRIAD
%   attitude initialisation and returns the Task 5 results struct. The
%   results are also saved in results/Result_IMU_GNSS_TRIAD.mat and the
%   summary line printed to the console.

if nargin == 0
    imuFile = 'IMU_X001.dat';
    gnssFile = 'GNSS_X001.csv';
    fprintf('[INFO] No files provided. Using defaults: %s, %s\n', imuFile, gnssFile);
elseif nargin ~= 2
    error('Usage: TRIAD(''IMUFILE'',''GNSSFILE'') or TRIAD() for defaults');
end

% Resolve to full paths so the function works from any directory. The helper
% GET_DATA_FILE searches IMU_MATLAB/data first and then the repository root.
imuFile  = get_data_file(imuFile);
gnssFile = get_data_file(gnssFile);

Task_1(imuFile, gnssFile, 'TRIAD');
Task_2(imuFile, gnssFile, 'TRIAD');
Task_3(imuFile, gnssFile, 'TRIAD');
Task_4(imuFile, gnssFile, 'TRIAD');
Task_5(imuFile, gnssFile, 'TRIAD');

[~, imuName]  = fileparts(imuFile);
[~, gnssName] = fileparts(gnssFile);
res_file = fullfile('results', sprintf('%s_%s_TRIAD_task5_results.mat', ...
    imuName, gnssName));
if ~isfile(res_file)
    error('Expected Task 5 results %s not found', res_file);
end
result = load(res_file);

save(fullfile('results', sprintf('Result_%s_%s_TRIAD.mat', imuName, gnssName)), ...
    '-struct', 'result');

sum_file = fullfile('results', 'IMU_GNSS_summary.txt');
if isfile(sum_file)
    lines = splitlines(fileread(sum_file));
    if ~isempty(lines)
        disp('--- TRIAD Method Summary ---');
        disp(lines{end-1});
    end
end
end
