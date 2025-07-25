%RUN_ALL_DATASETS Batch process all IMU/GNSS pairs with all methods
%   RUN_ALL_DATASETS() searches for IMU_X*.dat and GNSS_X*.csv files in the
%   current folder, pairs them by index and executes the full pipeline
%   (Tasks 1--5) for the methods TRIAD, Davenport and SVD. After each run
%   the Task 5 results structure is loaded into the base workspace under
%   a variable named result_IMU_Xxxx_GNSS_Xxxx_METHOD and also written to
%   output_matlab/<variable>.mat.

imu_files = dir('IMU_X*.dat');
gnss_files = dir('GNSS_X*.csv');
methods = {'TRIAD','Davenport','SVD'};

if numel(imu_files) ~= numel(gnss_files)
    error('Number of IMU and GNSS files must match.');
end

if ~exist('output_matlab','dir')
    mkdir('output_matlab');
end

for i = 1:numel(imu_files)
    imuFile  = imu_files(i).name;
    gnssFile = gnss_files(i).name;
    [~, imuName, ~]  = fileparts(imuFile);
    [~, gnssName, ~] = fileparts(gnssFile);

    for j = 1:numel(methods)
        method = methods{j};
        fprintf('Running %s + %s with %s...\n', imuFile, gnssFile, method);
        main(imuFile, gnssFile, method);

        result_file = fullfile('output_matlab', sprintf('%s_%s_%s_task5_results.mat',...
            imuName, gnssName, method));
        if ~isfile(result_file)
            warning('Missing expected output %s', result_file);
            continue;
        end
        result = load(result_file);
        var_name = sprintf('result_%s_%s_%s', imuName, gnssName, method);
        assignin('base', var_name, result);
        save(fullfile('output_matlab',[var_name '.mat']), '-struct', 'result');
        fprintf('Saved %s\n', fullfile('output_matlab',[var_name '.mat']));
    end
end

fprintf('All datasets complete.\n');

