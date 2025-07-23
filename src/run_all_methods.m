function run_all_methods(varargin)
%RUN_ALL_METHODS  MATLAB equivalent of run_all_methods.py
%   RUN_ALL_METHODS() processes all default IMU/GNSS pairs with the
%   attitude initialisation methods TRIAD, SVD and Davenport. Results are
%   saved under the 'results' directory and a summary table is written to
%   results/summary.csv.
%
%   This is a simplified translation of the Python script
%   ``src/run_all_methods.py`` and does not yet implement YAML config
%   parsing or the optional overlay/evaluation steps.
%
%   Example:
%       run_all_methods
%
%   Optional name/value pairs:
%       'config'           - YAML configuration file (not implemented)
%       'no_plots'         - true to skip plotting
%       'show_measurements'- include measurements in overlay plots

% Default datasets and methods
DATASETS = {
    'IMU_X001.dat', 'GNSS_X001.csv';
    'IMU_X002.dat', 'GNSS_X002.csv';
    'IMU_X003.dat', 'GNSS_X002.csv';
};
METHODS = {'TRIAD','SVD','Davenport'};

p = inputParser;
addParameter(p,'config','',@ischar);
addParameter(p,'no_plots',false,@islogical);
addParameter(p,'show_measurements',false,@islogical);
parse(p,varargin{:});
cfg_file = p.Results.config; %#ok<NASGU>

if ~exist('results','dir')
    mkdir('results');
end

summary = {};
for i = 1:size(DATASETS,1)
    imu_file  = DATASETS{i,1};
    gnss_file = DATASETS{i,2};
    for j = 1:numel(METHODS)
        method = METHODS{j};
        tag = sprintf('%s_%s_%s', erase(imu_file,'.dat'), erase(gnss_file,'.csv'), method);
        fprintf('>> %s\n', tag);
        data = read_data(imu_file, gnss_file);
        result = process_data(data, method);
        save_results(result, tag);
        if isfield(result,'fused_pos') && isfield(result,'gnss_pos')
            err = result.fused_pos - result.gnss_pos;
            rmse_pos = sqrt(mean(sum(err.^2,2)));
            final_pos = norm(err(end,:));
        else
            rmse_pos = NaN;
            final_pos = NaN;
        end
        summary(end+1,:) = {imu_file, method, rmse_pos, final_pos}; %#ok<AGROW>
    end
end

T = cell2table(summary, 'VariableNames', {'Dataset','Method','RMSEpos_m','EndErr_m'});
writetable(T, fullfile('results','summary.csv'));

fprintf('\n');
disp(T);
end
