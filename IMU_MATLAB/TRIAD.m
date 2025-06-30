function results = TRIAD(imu_paths, gnss_paths, truthFile)
%TRIAD  Run the pipeline using only the TRIAD method.
%   RESULTS = TRIAD(IMU_PATHS, GNSS_PATHS, TRUTHFILE) executes Tasks 1--5
%   with the TRIAD attitude initialisation for one or more IMU/GNSS file
%   pairs. When TRUTHFILE is provided it is forwarded to Task_5 so the
%   comparison plots include the reference trajectory. When called without
%   arguments all bundled sample datasets are processed. Single file names
%   or cell arrays of names are accepted. The Task 5 results for each pair
%   are returned as a struct (or a cell array of structs) and saved in
%   results/Result_<IMU>_<GNSS>_TRIAD.mat.

if nargin == 0
    imu_paths = {'IMU_X001.dat','IMU_X002.dat','IMU_X003.dat'};
    gnss_paths = {'GNSS_X001.csv','GNSS_X002.csv','GNSS_X002.csv'};
    truthFile = '';
elseif nargin == 2
    truthFile = '';
elseif nargin ~= 3
    error('Usage: TRIAD(''IMU_PATH'',''GNSS_PATH'',''truthFile'') or TRIAD() for defaults');
end

% Convert to cell arrays for uniform handling
if ischar(imu_paths) || isstring(imu_paths)
    imu_paths = {char(imu_paths)};
end
if ischar(gnss_paths) || isstring(gnss_paths)
    gnss_paths = {char(gnss_paths)};
end

if numel(imu_paths) ~= numel(gnss_paths)
    error('Number of IMU and GNSS files must match.');
end

results = cell(1, numel(imu_paths));

for k = 1:numel(imu_paths)
    imu_file  = get_data_file(imu_paths{k});
    gnss_file = get_data_file(gnss_paths{k});

    Task_1(imu_file, gnss_file, 'TRIAD');
    Task_2(imu_file, gnss_file, 'TRIAD');
    Task_3(imu_file, gnss_file, 'TRIAD');
    Task_4(imu_file, gnss_file, 'TRIAD');
    Task_5(imu_file, gnss_file, 'TRIAD', [], truthFile);

    [~, imuName]  = fileparts(imu_file);
    [~, gnssName] = fileparts(gnss_file);
    res_file = fullfile('results', sprintf('%s_%s_TRIAD_task5_results.mat', ...
        imuName, gnssName));
    if ~isfile(res_file)
        error('Expected Task 5 results %s not found', res_file);
    end
    r = load(res_file);
    save(fullfile('results', sprintf('Result_%s_%s_TRIAD.mat', ...
         imuName, gnssName)), '-struct', 'r');

    sum_file = fullfile('results', 'IMU_GNSS_summary.txt');
    if isfile(sum_file)
        lines = splitlines(fileread(sum_file));
        if ~isempty(lines)
            disp('--- TRIAD Method Summary ---');
            disp(lines{end-1});
        end
    end

    results{k} = r;
end

if numel(results) == 1
    results = results{1};
end
end
