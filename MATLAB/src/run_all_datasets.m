%RUN_ALL_DATASETS Batch process all IMU/GNSS pairs with all methods
%   RUN_ALL_DATASETS() searches for IMU_X*.dat and GNSS_X*.csv files in the
%   current folder, pairs them by dataset ID (e.g., X001) and executes the full
%   pipeline (Tasks 1--5) for the methods TRIAD, Davenport and SVD. Missing
%   IMU/GNSS pairs are skipped with a warning. After each run
%   the Task 5 results structure is loaded into the base workspace under
%   a variable named result_IMU_Xxxx_GNSS_Xxxx_METHOD and also written to

%   ``results/<variable>.mat`` within the directory returned by ``get_results_dir()``.

format long g % mirror Python full-precision printing

imu_files = dir('IMU_X*.dat');
gnss_files = dir('GNSS_X*.csv');
methods = {'TRIAD','Davenport','SVD'};
% Build dataset pairs only where both IMU and GNSS files exist
imu_ids = regexp({imu_files.name}, 'IMU_(X\d+)\.dat', 'tokens');
imu_ids = cellfun(@(c)c{1}, imu_ids, 'UniformOutput', false);
gnss_ids = regexp({gnss_files.name}, 'GNSS_(X\d+)\.csv', 'tokens');
gnss_ids = cellfun(@(c)c{1}, gnss_ids, 'UniformOutput', false);
all_ids = unique([imu_ids, gnss_ids]);
pairs = {};
for k = 1:numel(all_ids)
    id = all_ids{k};
    imuFile  = sprintf('IMU_%s.dat', id);
    gnssFile = sprintf('GNSS_%s.csv', id);
    if isfile(imuFile) && isfile(gnssFile)
        pairs(end+1,:) = {imuFile, gnssFile}; %#ok<AGROW>
    else
        warning('Skipping dataset %s: missing IMU or GNSS file.', id);
    end
end

results_dir = get_results_dir();
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end

for i = 1:size(pairs,1)
    imuFile  = pairs{i,1};
    gnssFile = pairs{i,2};
    [~, imuName, ~]  = fileparts(imuFile);
    [~, gnssName, ~] = fileparts(gnssFile);

    for j = 1:numel(methods)
        method = methods{j};
        fprintf('Running %s + %s with %s...\n', imuFile, gnssFile, method);
        main(imuFile, gnssFile, method);

        result_file = fullfile(results_dir, sprintf('%s_%s_%s_task5_results.mat',...
            imuName, gnssName, method));
        if ~isfile(result_file)
            warning('Missing expected output %s', result_file);
            continue;
        end
        try
            result = load(result_file);
        catch ME
            warning('Failed to load %s: %s', result_file, ME.message);
            continue; % skip MATLAB export if result file is bad
        end
        var_name = sprintf('result_%s_%s_%s', imuName, gnssName, method);
        assignin('base', var_name, result);
        save(fullfile(results_dir,[var_name '.mat']), '-struct', 'result');
        fprintf('Saved %s\n', fullfile(results_dir,[var_name '.mat']));
    end
end

fprintf('All datasets complete.\n');

