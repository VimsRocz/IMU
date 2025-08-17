function run_all_datasets_matlab(method)
%RUN_ALL_DATASETS_MATLAB  Pure MATLAB batch runner for all datasets
%   RUN_ALL_DATASETS_MATLAB(METHOD) enumerates the IMU/GNSS pairs in the
%   repository and executes Task_1 through Task_5 for the provided
%   initialisation METHOD. When METHOD is omitted the function iterates over
%   {'TRIAD','Davenport','SVD'}. The final Task 5 results are saved as
%   <IMU>_<GNSS>_<METHOD>_kf_output.mat in the results directory. plot_results
%   is called on each file to recreate the standard figures. A summary table
%   mirroring ``src/run_all_datasets.py`` is printed and saved as
%   ``results/summary.csv`` within the directory returned by
%   ``get_results_dir()``.
%
% Usage:

%   run_all_datasets_matlab(method)

format long g % mirror Python full-precision printing

if nargin < 1 || isempty(method)
    method_list = {'TRIAD','Davenport','SVD'};
elseif ischar(method)
    method_list = {method};
else
    method_list = method;
end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
% Ensure this file's folder is on the MATLAB path so Task_* functions are
% found even after changing directories. This mirrors the behaviour of the
% Python batch runner which uses module imports.
addpath(here);
% Ensure "results" directory is under the repository root regardless of the
% caller's current working directory.
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

resultsDir = get_results_dir();
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end



fusion_results = struct('dataset',{},'method',{},'rmse_pos',{},'final_pos',{},...
    'rms_resid_pos',{},'max_resid_pos',{},'rms_resid_vel',{},'max_resid_vel',{},...
    'accel_bias',{},'gyro_bias',{},'grav_err_mean',{},'grav_err_max',{},...
    'omega_err_mean',{},'omega_err_max',{},'ZUPT_count',{},'runtime',{});

for k = 1:size(pairs,1)
    imu  = fullfile(root, pairs{k,1});
    gnss = fullfile(root, pairs{k,2});

    for m = 1:numel(method_list)
        method = method_list{m};
        fprintf('Processing %s with %s using %s...\n', pairs{k,1}, pairs{k,2}, method);

        start_t = tic;
        Task_1(imu, gnss, method);
        Task_2(imu, gnss, method);
        Task_3(imu, gnss, method);
        Task_4(imu, gnss, method);
        Task_5(imu, gnss, method);
        runtime = toc(start_t);

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
            % After saving the standard MAT output, compare with the
            % corresponding Python NPZ file when available. The helper
            % script prints RMSE statistics to highlight any parity
            % issues between implementations.
            try
                check_latest_python_matlab;
            catch ME
                warning('Parity check failed: %s', ME.message);
            end
            cand = fullfile(root, 'STATE_X001.txt');
            if isfile(cand)
                fprintf('Starting Task 6 for %s + %s ...\n', imuStem, gnssStem);
                try
                    tag = sprintf('%s_%s_%s', imuStem, gnssStem, method);
                    Task_6(task5File, cand, tag);
                catch ME
                    warning('Task 6 failed: %s', ME.message);
                end
                fprintf('Starting Task 7 for %s + %s ...\n', imuStem, gnssStem);
                try
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

        summary_file = fullfile(resultsDir, sprintf('%s_%s_%s_summary.txt', ...
            imuStem, gnssStem, method));
        metrics = parse_summary_file(summary_file);
        tok = regexp(imuStem,'(X\d+)','tokens','once');
        if ~isempty(tok); metrics.dataset = tok{1}; else; metrics.dataset = imuStem; end
        metrics.method = method;
        metrics.runtime = runtime;
        fusion_results(end+1) = metrics; %#ok<AGROW>
    end
end

T = struct2table(fusion_results);
writetable(T, fullfile(resultsDir,'summary.csv'));
disp(T);

fprintf('All datasets processed.\n');
files = dir(fullfile(resultsDir, '*.mat'));
fprintf('Generated %d result files in %s:\n', numel(files), resultsDir);
for f = files'; fprintf('  %s\n', f.name); end
end


function metrics = parse_summary_file(file)
    metrics = struct('rmse_pos',NaN,'final_pos',NaN,'rms_resid_pos',NaN,'max_resid_pos',NaN,...
        'rms_resid_vel',NaN,'max_resid_vel',NaN,'accel_bias',NaN,'gyro_bias',NaN,...
        'grav_err_mean',NaN,'grav_err_max',NaN,'omega_err_mean',NaN,'omega_err_max',NaN,...
        'ZUPT_count',NaN);
    if exist(file,'file')
        txt = strtrim(fileread(file));
        tokens = regexp(txt,'(\w+)=([^\s]+)','tokens');
        for i=1:numel(tokens)
            key = tokens{i}{1};
            val = regexprep(tokens{i}{2},'[a-zA-Z/]+','');
            num = str2double(val);
            if ~isnan(num)
                metrics.(key) = num;
            end
        end
    end
end
