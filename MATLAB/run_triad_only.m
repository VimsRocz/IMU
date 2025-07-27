%% RUN_TRIAD_ONLY  Process all datasets using the TRIAD method only
% This helper mirrors the behaviour of ``src/run_triad_only.py``.
% It first executes the full MATLAB pipeline for the TRIAD method via
% ``run_all_datasets_matlab`` and then aggregates the Task 5 summaries into a
% concise ``results/summary.csv`` within the directory returned by
% ``get_results_dir()``.
%
% Usage:
%   run_triad_only
%
% The routine parses ``results/IMU_GNSS_summary.txt`` for lines beginning with
% ``[SUMMARY]`` and extracts the metrics for the TRIAD runs.  It also
% approximates the runtime from the time vector saved in
% ``<IMU>_<GNSS>_TRIAD_task5_results.mat``.
%
% See also: run_all_datasets_matlab, run_method_only

function run_triad_only
    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    results_dir = get_results_dir();
    if ~exist(results_dir, 'dir'); mkdir(results_dir); end

    % Run the complete pipeline for the TRIAD method
    run_all_datasets_matlab('TRIAD');

    summary_file = fullfile(results_dir, 'IMU_GNSS_summary.txt');
    if ~isfile(summary_file)
        warning('Summary file %s not found. No results to summarise.', summary_file);
        return
    end

    lines = splitlines(fileread(summary_file));
    results = struct('Dataset', {}, 'Method', {}, 'RMSEpos_m', {}, ...
        'EndErr_m', {}, 'RMSresidPos_m', {}, 'MaxresidPos_m', {}, ...
        'RMSresidVel_mps', {}, 'MaxresidVel_mps', {}, 'Runtime_s', {});

    for i = 1:numel(lines)
        line = strtrim(lines{i});
        if isempty(line) || ~contains(line, '[SUMMARY]') || ~contains(line, 'method=TRIAD')
            continue
        end

        tokens = regexp(line, '(\w+)=\s*([^\s]+)', 'tokens');
        kv = containers.Map();
        for j = 1:numel(tokens)
            kv(tokens{j}{1}) = tokens{j}{2};
        end

        imu_name  = '';
        gnss_name = '';
        method    = '';
        if isKey(kv, 'imu');   imu_name  = kv('imu');   end
        if isKey(kv, 'gnss');  gnss_name = kv('gnss');  end
        if isKey(kv, 'method'); method   = kv('method'); end
        [~, imu_stem, ~]  = fileparts(imu_name);
        [~, gnss_stem, ~] = fileparts(gnss_name);

        rmse_pos      = NaN;
        final_pos     = NaN;
        rms_resid_pos = NaN;
        max_resid_pos = NaN;
        rms_resid_vel = NaN;
        max_resid_vel = NaN;
        if isKey(kv, 'rmse_pos');      rmse_pos      = str2double(erase(kv('rmse_pos'), 'm')); end
        if isKey(kv, 'final_pos');     final_pos     = str2double(erase(kv('final_pos'), 'm')); end
        if isKey(kv, 'rms_resid_pos'); rms_resid_pos = str2double(erase(kv('rms_resid_pos'), 'm')); end
        if isKey(kv, 'max_resid_pos'); max_resid_pos = str2double(erase(kv('max_resid_pos'), 'm')); end
        if isKey(kv, 'rms_resid_vel'); rms_resid_vel = str2double(erase(kv('rms_resid_vel'), 'm')); end
        if isKey(kv, 'max_resid_vel'); max_resid_vel = str2double(erase(kv('max_resid_vel'), 'm')); end

        tag = sprintf('%s_%s_%s', imu_stem, gnss_stem, method);
        task5_file = fullfile(results_dir, [tag '_task5_results.mat']);
        runtime_s = NaN;
        if isfile(task5_file)
            S = load(task5_file, 'time');
            if isfield(S, 'time') && numel(S.time) >= 2
                runtime_s = S.time(end) - S.time(1);
            end
        end

        idx = numel(results) + 1;
        results(idx).Dataset         = imu_stem;
        results(idx).Method          = method;
        results(idx).RMSEpos_m       = rmse_pos;
        results(idx).EndErr_m        = final_pos;
        results(idx).RMSresidPos_m   = rms_resid_pos;
        results(idx).MaxresidPos_m   = max_resid_pos;
        results(idx).RMSresidVel_mps = rms_resid_vel;
        results(idx).MaxresidVel_mps = max_resid_vel;
        results(idx).Runtime_s       = runtime_s;
    end

    if isempty(results)
        warning('No TRIAD summary lines found in %s.', summary_file);
        return
    end

    T = struct2table(results);
    fprintf('\nOverall performance summary (TRIAD)\n');
    disp(T);

    csv_path = fullfile(results_dir, 'summary.csv');
    try
        writetable(T, csv_path);
        fprintf('Summary written to %s\n', csv_path);
    catch ME
        warning('Failed to write summary CSV: %s', ME.message);
    end
end
