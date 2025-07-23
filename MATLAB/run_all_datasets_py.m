function run_all_datasets_py(varargin)
%RUN_ALL_DATASETS_PY  MATLAB equivalent of run_all_datasets.py
%   RUN_ALL_DATASETS_PY mirrors the Python batch runner. It processes all
%   IMU/GNSS pairs with each attitude initialisation method and prints a
%   summary table. Results and logs are written to the 'results' folder.
%
%   Usage:
%       run_all_datasets_py
%       run_all_datasets_py('datasets','X001,X003','method','TRIAD','verbose',true)
%
%   Optional Name-Value pairs:
%       'verbose' - show debugging information (default false)
%       'datasets' - comma separated dataset IDs or 'ALL' (default 'ALL')
%       'method'  - 'TRIAD','Davenport','SVD' or 'ALL' (default 'ALL')
%       'config'  - YAML configuration file listing datasets and methods
%
%   This function assumes GNSS_IMU_Fusion.m produces files named
%   <IMU>_<GNSS>_<METHOD>_kf_output.npz in the current results directory.
%   Metrics are derived from these files to mirror the Python summary.

% Default dataset list and methods
DEFAULT_DATASETS = {
    'IMU_X001.dat', 'GNSS_X001.csv';
    'IMU_X002.dat', 'GNSS_X002.csv';
    'IMU_X003.dat', 'GNSS_X002.csv';
};
DEFAULT_METHODS = {'TRIAD','Davenport','SVD'};

p = inputParser;
addParameter(p,'verbose',false,@islogical);
addParameter(p,'datasets','ALL',@ischar);
addParameter(p,'method','ALL',@ischar);
addParameter(p,'config','',@ischar);
parse(p,varargin{:});
verbose  = p.Results.verbose;
sel_sets = p.Results.datasets;
sel_method = p.Results.method;
config_file = p.Results.config;

DATASETS = DEFAULT_DATASETS;
METHODS  = DEFAULT_METHODS;

% Optional YAML configuration
if ~isempty(config_file)
    if exist(config_file,'file')
        cfg = yamlread(config_file);
        if isfield(cfg,'datasets')
            DATASETS = cell(size(cfg.datasets,1),2);
            for i=1:numel(cfg.datasets)
                DATASETS{i,1} = cfg.datasets{i}.imu;
                DATASETS{i,2} = cfg.datasets{i}.gnss;
            end
        end
        if isfield(cfg,'methods'); METHODS = cfg.methods; end
    else
        error('Config file not found: %s', config_file);
    end
end

% Filter datasets selection
if ~strcmpi(sel_sets,'ALL')
    ids = regexp(strsplit(sel_sets,','),'\w+','match');
    ids = [ids{:}];
    keep = false(size(DATASETS,1),1);
    for k=1:size(DATASETS,1)
        tok = regexp(DATASETS{k,1},'IMU_(X\d+)','tokens','once');
        if ~isempty(tok)
            keep(k) = any(strcmpi(tok{1}, ids));
        end
    end
    DATASETS = DATASETS(keep,:);
end

% Filter methods
if ~strcmpi(sel_method,'ALL')
    METHODS = {sel_method};
end

cases = {};
for i=1:size(DATASETS,1)
    for j=1:numel(METHODS)
        cases(end+1,:) = {DATASETS{i,1}, DATASETS{i,2}, METHODS{j}}; %#ok<AGROW>
    end
end

results_dir = fullfile(pwd,'results');
if ~exist(results_dir,'dir'); mkdir(results_dir); end
log_dir = fullfile(fileparts(mfilename('fullpath')),'logs');
if ~exist(log_dir,'dir'); mkdir(log_dir); end

fusion_results = struct('dataset',{},'method',{},'rmse_pos',{},'final_pos',{},...
    'rms_resid_pos',{},'max_resid_pos',{},'rms_resid_vel',{},'max_resid_vel',{},...
    'runtime',{});

for idx = 1:size(cases,1)
    imu  = cases{idx,1};
    gnss = cases{idx,2};
    method = cases{idx,3};

    if verbose
        fprintf('==== DEBUG: File Pairing ====\n');
        fprintf('IMU file: %s\n', imu);
        fprintf('GNSS file: %s\n', gnss);
        Tgnss = readtable(gnss);
        imu_data = readmatrix(imu);
        fprintf('GNSS shape: %s\n', mat2str(size(Tgnss)));
        fprintf('IMU shape: %s\n', mat2str(size(imu_data)));
        fprintf('GNSS time [start end]: %.2f %.2f\n', Tgnss.Posix_Time(1), Tgnss.Posix_Time(end));
        fprintf('IMU time [start end]: %.2f %.2f\n', imu_data(1,1), imu_data(end,1));
        fprintf('Any NaNs in GNSS? %d\n', sum(sum(ismissing(Tgnss))));
        fprintf('Any NaNs in IMU? %d\n', sum(isnan(imu_data(:))));
        fprintf('GNSS Head:\n'); disp(head(Tgnss));
        fprintf('IMU Head:\n'); disp(imu_data(1:min(5,end),:));
        fprintf('============================\n');
    end

    tic;
    run_one(imu, gnss, method, verbose, results_dir, log_dir);
    runtime = toc;
    metrics = compute_metrics(results_dir, imu, gnss, method);
    metrics.runtime = runtime;
    fusion_results(end+1) = metrics; %#ok<AGROW>
end

% Sort results by dataset then method
[~, order] = sortrows({fusion_results.dataset}.');
fusion_results = fusion_results(order);

% Display summary table
rows = cell(numel(fusion_results),8);
for k=1:numel(fusion_results)
    r = fusion_results(k);
    rows(k,:) = {r.dataset, r.method, r.rmse_pos, r.final_pos, ...
        r.rms_resid_pos, r.max_resid_pos, r.rms_resid_vel, r.max_resid_vel, r.runtime};
end
header = {'Dataset','Method','RMSEpos [m]','End-Error [m]', ...
    'RMSresidPos [m]','MaxresidPos [m]','RMSresidVel [m/s]','MaxresidVel [m/s]','Runtime [s]'};
T = cell2table(rows,'VariableNames',header);

fprintf('\n');
disp(T);

writetable(T, fullfile(results_dir,'summary.csv'));
end

function run_one(imu, gnss, method, verbose, results_dir, log_dir)
%RUN_ONE Execute GNSS_IMU_Fusion and save standard MAT output
    ts = datestr(now,'yyyymmdd_HHMMSS');
    log_file = fullfile(log_dir, sprintf('%s_%s_%s_%s.log', imu, gnss, method, ts));
    diary(log_file); %#ok<DIARY>
    try
        GNSS_IMU_Fusion(imu, gnss, method);
    catch ME
        diary off; rethrow(ME);
    end
    diary off;
    % Convert NPZ to MAT if produced
    npz_name = sprintf('%s_%s_%s_kf_output.npz', erase(imu,'.dat'), erase(gnss,'.csv'), method);
    npz_path = fullfile(results_dir, npz_name);
    if exist(npz_path,'file')
        data = py.numpy.load(npz_path, pyargs('allow_pickle',true));
        keys = cell(data.keys); S = struct();
        for i=1:numel(keys)
            S.(keys{i}) = double(data{keys{i}});
        end
        save(fullfile(results_dir, [erase(npz_name,'.npz') '.mat']), '-struct', 'S');
    end
end

function metrics = compute_metrics(results_dir, imu, gnss, method)
%COMPUTE_METRICS Derive basic performance metrics from NPZ results
    metrics = struct('dataset',erase(regexprep(imu,'IMU_(X\d+)\.dat','$1'),'.'), ...
                     'method',method, 'rmse_pos',NaN,'final_pos',NaN, ...
                     'rms_resid_pos',NaN,'max_resid_pos',NaN, ...
                     'rms_resid_vel',NaN,'max_resid_vel',NaN);
    npz_file = fullfile(results_dir, sprintf('%s_%s_%s_kf_output.npz', ...
        erase(imu,'.dat'), erase(gnss,'.csv'), method));
    if ~exist(npz_file,'file'); return; end
    data = py.numpy.load(npz_file, pyargs('allow_pickle',true));
    if isKey(data,'residual_pos') && isKey(data,'residual_vel')
        res_pos = double(data{'residual_pos'});
        res_vel = double(data{'residual_vel'});
        norm_pos = vecnorm(res_pos,2,2);
        norm_vel = vecnorm(res_vel,2,2);
        metrics.rmse_pos = sqrt(mean(norm_pos.^2));
        metrics.final_pos = norm_pos(end);
        metrics.rms_resid_pos = metrics.rmse_pos;
        metrics.max_resid_pos = max(norm_pos);
        metrics.rms_resid_vel = sqrt(mean(norm_vel.^2));
        metrics.max_resid_vel = max(norm_vel);
    end
end

function s = yamlread(file)
%YAMLREAD Simple YAML loader returning a struct
    txt = fileread(file);
    pyyaml = py.importlib.import_module('yaml');
    pyobj = pyyaml.safe_load(txt);
    s = struct(pyobj);
end
