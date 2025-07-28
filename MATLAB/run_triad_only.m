%RUN_TRIAD_ONLY  Process dataset X002 using the TRIAD method.
%   This script mirrors ``src/run_triad_only.py`` but follows the
%   ``run_all_methods.m`` approach where Tasks 1--7 are executed
%   sequentially in MATLAB. Dataset files are referenced directly from the
%   repository root and all outputs are written to ``results/`` using the
%   standard naming convention. The printed rotation matrix should match
%   the Python value
%   ``[0.2336 -0.0454 0.9713; 0.0106 0.9990 0.0441; -0.9723 0.0000 0.2339]``
%   up to numerical precision.
%
%   Usage:
%       run_triad_only


%% Resolve helper path
st = dbstack('-completenames');
if ~isempty(st)
    script_dir = fileparts(st(1).file);
else
    script_dir = fileparts(mfilename('fullpath'));
end
addpath(script_dir);

method    = 'TRIAD';
root_dir  = fileparts(fileparts(mfilename('fullpath')));

results_dir = get_results_dir();
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

fprintf('%s Running TRIAD on all datasets (MATLAB pipeline)\n', char(hex2dec('25B6')));
fprintf('Ensured ''%s'' directory exists.\n', results_dir);
start_t = tic;

run_all_datasets_matlab(method);

% Parse summary lines from Task 5
sum_file = fullfile(results_dir,'IMU_GNSS_summary.txt');
if isfile(sum_file)
    lines = strtrim(splitlines(fileread(sum_file)));
    lines = lines(~cellfun('isempty',lines));
else
    lines = {};
end
datasets = {}; metrics = [];
for i=1:numel(lines)
    tok = regexp(lines{i},'(\w+)=([^\s]+)','tokens');
    kv = struct();
    for t=1:numel(tok)
        kv.(tok{t}{1}) = tok{t}{2};
    end
    tag = sprintf('%s_%s_%s', erase(kv.imu,'.dat'), erase(kv.gnss,'.csv'), kv.method);
    matfile = fullfile(results_dir, sprintf('%s_task5_results.mat', tag));
    runtime = NaN;
    if isfile(matfile)
        T = load(matfile,'time');
        if isfield(T,'time'); runtime = T.time(end)-T.time(1); end
    end
    datasets{end+1,1} = regexp(kv.imu,'(X\d+)','tokens','once');
    if isempty(datasets{end}); datasets{end} = kv.imu; else; datasets{end} = datasets{end}{1}; end
    metrics(end+1,:) = [str2double(kv.rmse_pos(1:end-1)), str2double(kv.final_pos(1:end-1)), ...
        str2double(kv.rms_resid_pos(1:end-1)), str2double(kv.max_resid_pos(1:end-1)), ...
        str2double(kv.rms_resid_vel(1:end-1)), str2double(kv.max_resid_vel(1:end-1)), runtime];
end
T = table(datasets, repmat({method},numel(datasets),1), metrics(:,1), metrics(:,2), ...
    metrics(:,3), metrics(:,4), metrics(:,5), metrics(:,6), metrics(:,7), ...
    'VariableNames',{'Dataset','Method','RMSEpos_m','EndErr_m','RMSresidPos_m','MaxresidPos_m','RMSresidVel_mps','MaxresidVel_mps','Runtime_s'});
writetable(T, fullfile(results_dir,'summary.csv'));
disp(T);

elapsed_s = toc(start_t);
fprintf('Runtime %.2f s\n', elapsed_s);
disp('TRIAD batch processing complete');
