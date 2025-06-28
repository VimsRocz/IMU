function result = TRIAD(imu_path, gnss_path)
%TRIAD Run the pipeline using only the TRIAD method.
%   RESULT = TRIAD(IMU_PATH, GNSS_PATH) executes Tasks 1--5 with the TRIAD
%   attitude initialisation and returns the Task 5 results struct. The
%   results are also saved in results/Result_IMU_GNSS_TRIAD.mat and the
%   summary line printed to the console.

if nargin == 0
    imu_path = 'IMU_X001.dat';
    gnss_path = 'GNSS_X001.csv';
    fprintf('[INFO] No files provided. Using defaults: %s, %s\n', imu_path, gnss_path);
elseif nargin ~= 2
    error('Usage: TRIAD(''IMU_PATH'',''GNSS_PATH'') or TRIAD() for defaults');
end

% Resolve to full paths so the function works from any directory. The helper
% GET_DATA_FILE searches IMU_MATLAB/data first and then the repository root.
imu_path  = get_data_file(imu_path);
gnss_path = get_data_file(gnss_path);

Task_1(imu_path, gnss_path, 'TRIAD');
Task_2(imu_path, gnss_path, 'TRIAD');
Task_3(imu_path, gnss_path, 'TRIAD');
Task_4(imu_path, gnss_path, 'TRIAD');
Task_5(imu_path, gnss_path, 'TRIAD');

[~, imuName]  = fileparts(imu_path);
[~, gnssName] = fileparts(gnss_path);
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
