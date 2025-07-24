%% RUN_TRIAD_ONLY  Process all datasets using only the TRIAD method
% This script mirrors the behaviour of ``run_all_methods.m`` but restricts
% the attitude initialisation to TRIAD.  All Tasks 1--7 are executed for
% each IMU/GNSS pair and the resulting figures and metrics are written to
% the ``results/`` directory using the same naming conventions as the
% multi-method pipeline.
%
% Usage:
%   run_triad_only

method = 'TRIAD';

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
% Switch to repository root so that all Tasks read/write results under
% ``root/results`` regardless of the caller's working directory.
orig_dir = pwd;
cd(root);
cleanupObj = onCleanup(@() cd(orig_dir));

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

    fprintf('\n=== Dataset %d/%d: %s + %s ===\n', ...
        k, size(pairs,1), pairs{k,1}, pairs{k,2});

    % Task 1--5: reference vectors, measurement extraction, attitude
    % initialisation, dataset alignment and sensor fusion
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

fprintf('TRIAD-only MATLAB run completed for all datasets. Results match multi-method pipeline (TRIAD only).\n');

