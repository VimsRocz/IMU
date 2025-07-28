%RUN_TRIAD_ONLY  Batch processing of all datasets using the TRIAD method.
%   This script mirrors ``src/run_triad_only.py`` and orchestrates Tasks 1-7
%   for each available IMU/GNSS dataset pair.  Results are written to
%   ``results/<IMU>_<GNSS>_TRIAD/`` under the repository root.
%
%   Usage:
%       run_triad_only              %% runs all detected dataset pairs
%       run_triad_only('IMU_X002_GNSS_X002') %% run a single pair
%
%   The script prints progress messages before and after each task.  Errors
%   are reported but do not stop processing of remaining datasets.

function run_triad_only(dataset)
if nargin < 1
    dataset = 'all';
end
method = 'TRIAD';

% Repository root
here = fileparts(mfilename('fullpath'));
root = fileparts(here);

% Define known dataset pairs (mirrors Python pipeline)
pairs = {
    'IMU_X001.dat', 'GNSS_X001.csv';
    'IMU_X002.dat', 'GNSS_X002.csv';
    'IMU_X003.dat', 'GNSS_X002.csv';
};

% Filter by user request
if ~strcmpi(dataset,'all')
    pairs = pairs(strcmpi(sprintf('%s_%s',erase(pairs(:,1),'.dat'),erase(pairs(:,2),'.csv')), dataset),:);
    if isempty(pairs)
        error('Dataset %s not found.', dataset);
    end
end

base_results = get_results_dir();
if ~exist(base_results,'dir'); mkdir(base_results); end

summaries = struct('dataset',{},'rmse_pos',{},'final_pos',{},'rms_resid_pos',{},...
    'max_resid_pos',{},'rms_resid_vel',{},'max_resid_vel',{},'runtime',{});

for i=1:size(pairs,1)
    imu_file  = fullfile(root, pairs{i,1});
    gnss_file = fullfile(root, pairs{i,2});
    [~, imu_stem, ~]  = fileparts(imu_file);
    [~, gnss_stem, ~] = fileparts(gnss_file);
    tag = sprintf('%s_%s_%s', imu_stem, gnss_stem, method);
    dataset_tag = sprintf('%s_%s', imu_stem, gnss_stem);
    out_dir = fullfile(base_results, tag);
    if ~exist(out_dir,'dir'); mkdir(out_dir); end
    setenv('MATLAB_RESULTS_DIR', out_dir);
    fprintf('\n\u25B6 %s\n', tag); % triangle symbol
    start_t = tic;
    failed = false;
    try
        fprintf('Task 1...\n');
        Task_1(imu_file, gnss_file, method, dataset_tag);
        fprintf('Task 1 done.\n');
        fprintf('Task 2...\n');
        Task_2(imu_file, gnss_file, method, dataset_tag);
        fprintf('Task 2 done.\n');
        fprintf('Task 3...\n');
        Task_3(imu_file, gnss_file, method, dataset_tag);
        fprintf('Task 3 done.\n');
        fprintf('Task 4...\n');
        Task_4(imu_file, gnss_file, method, dataset_tag);
        fprintf('Task 4 done.\n');
        fprintf('Task 5...\n');
        Task_5(imu_file, gnss_file, method, dataset_tag);
        fprintf('Task 5 done.\n');
    catch ME
        fprintf('Error in Tasks 1-5 for %s: %s\n', tag, ME.message);
        failed = true;
    end

    task5_file = fullfile(out_dir, sprintf('Task5_%s_%s.mat', dataset_tag, method));
    truth_file = fullfile(root,'STATE_X001.txt');
    if ~failed && isfile(task5_file) && isfile(truth_file)
        try
            fprintf('Task 6...\n');
            Task_6(task5_file, truth_file, tag);
            fprintf('Task 6 done.\n');
        catch ME
            fprintf('Task 6 failed: %s\n', ME.message);
        end
        try
            fprintf('Task 7...\n');
            Task_7(task5_file, truth_file, tag);
            fprintf('Task 7 done.\n');
        catch ME
            fprintf('Task 7 failed: %s\n', ME.message);
        end
    end
    runtime = toc(start_t);
    summary_mat = fullfile(out_dir, sprintf('%s_task7_summary_ned.mat', tag));
    if isfile(summary_mat)
        S = load(summary_mat, 'summary');
        sum = S.summary;
        summaries(end+1) = struct('dataset',imu_stem,'rmse_pos',sum.rmse_pos,...
            'final_pos',sum.final_pos,'rms_resid_pos',sum.rms_resid_pos,...
            'max_resid_pos',sum.max_resid_pos,'rms_resid_vel',sum.rms_resid_vel,...
            'max_resid_vel',sum.max_resid_vel,'runtime',runtime); %#ok<AGROW>
    else
        summaries(end+1) = struct('dataset',imu_stem,'rmse_pos',NaN,'final_pos',NaN,...
            'rms_resid_pos',NaN,'max_resid_pos',NaN,'rms_resid_vel',NaN,'max_resid_vel',NaN,'runtime',runtime); %#ok<AGROW>
    end
    setenv('MATLAB_RESULTS_DIR','');
end

% Print summary table
if ~isempty(summaries)
    T = struct2table(summaries);
    T.Method = repmat({method},height(T),1);
    T = movevars(T,'Method','After','dataset');
    disp(T);
    writetable(T, fullfile(base_results,'summary.csv'));
end

end

