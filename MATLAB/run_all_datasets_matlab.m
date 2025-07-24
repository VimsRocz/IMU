function run_all_datasets_matlab(method)
%RUN_ALL_DATASETS_MATLAB  Pure MATLAB batch runner for all datasets
%   RUN_ALL_DATASETS_MATLAB(METHOD) enumerates the IMU/GNSS pairs in the
%   repository and executes Task_1 through Task_5 for the provided
%   initialisation METHOD.  The final Task 5 results are saved as
%   <IMU>_<GNSS>_<METHOD>_kf_output.mat in the results directory.
%   plot_results is called on each file to recreate the standard figures.

if nargin < 1 || isempty(method)
    method = 'TRIAD';
end

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
        % Always use the common STATE\_X001.txt trajectory as the reference
        % for Tasks 6 and 7 regardless of the IMU/GNSS filename.  This
        % ensures that evaluation is performed for every dataset even when
        % dataset-specific truth files are unavailable.
        cand = fullfile(root, 'STATE_X001.txt');
        if isfile(cand)
            fprintf('Starting Task 6 for %s + %s ...\n', imuStem, gnssStem);
            try
                Task_6(task5File, imu, gnss, cand);
            catch ME
                warning('Task 6 failed: %s', ME.message);
            end
            fprintf('Starting Task 7 for %s + %s ...\n', imuStem, gnssStem);
            try
                tag = sprintf('%s_%s_%s', imuStem, gnssStem, method);
                outDir = fullfile(resultsDir, 'task7', tag);
                summary = task7_fused_truth_error_analysis(outFile, cand, outDir);
                save(fullfile(outDir,'task7_summary.mat'), 'summary');
            catch ME
                warning('Task 7 failed: %s', ME.message);
            end
        end
    else
        warning('Missing %s', task5File);
    end
end

fprintf('All datasets processed.\n');
end
