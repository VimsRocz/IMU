function run_all_datasets_matlab()
%RUN_ALL_DATASETS_MATLAB  Pure MATLAB batch runner for all datasets
%   Enumerates the IMU/GNSS pairs in the repository and executes
%   Task_1 through Task_5 for the TRIAD method. The final Task 5
%   results are saved as <IMU>_<GNSS>_TRIAD_kf_output.mat in the
%   results directory. plot_results is called on each file to
%   recreate the standard figures.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);

% Prefer a dedicated Data folder if present
dataDir = fullfile(root, 'Data');
if ~exist(dataDir, 'dir')
    dataDir = root;
end

pairs = {
    'IMU_X001.dat', 'GNSS_X001.csv';
    'IMU_X002.dat', 'GNSS_X002.csv';
    'IMU_X003.dat', 'GNSS_X002.csv';
};

resultsDir = fullfile(root, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

for k = 1:size(pairs,1)
    imu  = fullfile(dataDir, pairs{k,1});
    gnss = fullfile(dataDir, pairs{k,2});

    % Fallback to helper search if file not in dataDir
    if ~isfile(imu);  imu  = get_data_file(pairs{k,1});  end
    if ~isfile(gnss); gnss = get_data_file(pairs{k,2}); end

    fprintf('Processing %s with %s...\n', pairs{k,1}, pairs{k,2});

    Task_1(imu, gnss, 'TRIAD');
    Task_2(imu, gnss, 'TRIAD');
    Task_3(imu, gnss, 'TRIAD');
    Task_4(imu, gnss, 'TRIAD');
    Task_5(imu, gnss, 'TRIAD');

    [~, imuStem, ~]  = fileparts(pairs{k,1});
    [~, gnssStem, ~] = fileparts(pairs{k,2});
    task5File = fullfile(resultsDir, sprintf('%s_%s_TRIAD_task5_results.mat', ...
        imuStem, gnssStem));
    outFile  = fullfile(resultsDir, sprintf('%s_%s_TRIAD_kf_output.mat', ...
        imuStem, gnssStem));
    if isfile(task5File)
        S = load(task5File);
        save(outFile, '-struct', 'S');
        plot_results(outFile);
        stateName = [strrep(imuStem,'IMU','STATE') '.txt'];
        cand = fullfile(dataDir, stateName);
        if ~isfile(cand)
            cand = fullfile(root, stateName);
        end
        if isfile(cand)
            try
                Task_6(imu, gnss, 'TRIAD');
            catch ME
                fprintf('Task_6 skipped: %s\n', ME.message);
            end
        end
    else
        warning('Missing %s', task5File);
    end
end

fprintf('All datasets processed.\n');
end
