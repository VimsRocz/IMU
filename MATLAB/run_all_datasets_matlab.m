function run_all_datasets_matlab(method)
%RUN_ALL_DATASETS_MATLAB  Pure MATLAB batch runner for all datasets
%   RUN_ALL_DATASETS_MATLAB(METHOD) enumerates the IMU/GNSS pairs in the
%   repository and executes Task_1 through Task_5 for the selected
%   attitude initialisation METHOD (``'TRIAD'`` by default). The final Task 5
%   results are saved as <IMU>_<GNSS>_<METHOD>_kf_output.mat in the results
%   directory. ``plot_results`` is called on each file to recreate the standard
%   figures.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);

if nargin < 1 || isempty(method)
    method = 'TRIAD';
end

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

    fprintf('Processing %s with %s using %s...\n', pairs{k,1}, pairs{k,2}, method);

    Task_1(imu, gnss, method);
    Task_2(imu, gnss, method);
    Task_3(imu, gnss, method);
    Task_4(imu, gnss, method);
    Task_5(imu, gnss, method);

    [~, imuStem, ~]  = fileparts(pairs{k,1});
    [~, gnssStem, ~] = fileparts(pairs{k,2});
    task5File = fullfile(resultsDir, sprintf('%s_%s_%s_task5_results.mat', ...
        imuStem, gnssStem, method));
    outFile  = fullfile(resultsDir, sprintf('%s_%s_%s_kf_output.mat', ...
        imuStem, gnssStem, method));
    if isfile(task5File)
        S = load(task5File);
        save(outFile, '-struct', 'S');
        if exist('plot_results.m','file')
            plot_results(outFile);
        end
        stateName = [strrep(imuStem,'IMU','STATE') '.txt'];
        cand = fullfile(dataDir, stateName);
        if ~isfile(cand)
            cand = fullfile(root, stateName);
        end
        if isfile(cand)
            try
                Task_6(imu, gnss, method);
            catch ME
                fprintf('Task_6 skipped: %s\n', ME.message);
            end
            try
                dataset = regexp(imuStem,'(X\d+)','match','once');
                task7_ecef_residuals_plot(outFile, cand, dataset, resultsDir);
            catch ME
                fprintf('Task_7 skipped: %s\n', ME.message);
            end
        end
    else
        warning('Missing %s', task5File);
    end
end

fprintf('All datasets processed.\n');
end
