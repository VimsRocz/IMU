%RUN_ALL_DATASETS Batch process all IMU/GNSS pairs with all methods
%   RUN_ALL_DATASETS() resolves all IMU_X*.dat and GNSS_X*.csv files using
%   GET_DATA_FILE, pairs them by index and executes the full pipeline
%   (Tasks 1--5) for the methods TRIAD, Davenport and SVD. After each run
%   the Task 5 results structure is loaded into the base workspace under
%   a variable named result_IMU_Xxxx_GNSS_Xxxx_METHOD and also written to
%   results/<variable>.mat.

% resolve dataset locations independent of the current folder
imu_files = get_data_file('IMU_X*.dat');
gnss_files = get_data_file('GNSS_X*.csv');
if ischar(imu_files); imu_files = {imu_files}; end
if ischar(gnss_files); gnss_files = {gnss_files}; end
imu_files = sort(imu_files);
gnss_files = sort(gnss_files);
methods = {'TRIAD','Davenport','SVD'};

if numel(imu_files) ~= numel(gnss_files)
    error('Number of IMU and GNSS files must match.');
end

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end
orig_dir = pwd;
cleanup = onCleanup(@() cd(orig_dir));
cd(script_dir);

for i = 1:numel(imu_files)
    imuFile  = imu_files{i};
    gnssFile = gnss_files{i};
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
        result = load(result_file);
        var_name = sprintf('result_%s_%s_%s', imuName, gnssName, method);
        assignin('base', var_name, result);
        save(fullfile(results_dir,[var_name '.mat']), '-struct', 'result');
        fprintf('Saved %s\n', fullfile(results_dir,[var_name '.mat']));
    end
end

fprintf('All datasets complete.\n');

