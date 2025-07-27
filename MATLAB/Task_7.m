function metrics = Task_7(fused_file, truth_file)
%TASK_7 Evaluate fused results against ground truth.
%   METRICS = TASK_7(FUSED_FILE, TRUTH_FILE) loads the fused estimator
%   result in FUSED_FILE and the reference trajectory TRUTH_FILE. The
%   existing residual-plot functions ``task7_ned_residuals_plot`` and
%   ``task7_ecef_residuals_plot`` are called to generate diagnostic
%   figures. Summary error metrics are computed via
%   ``task7_fused_truth_error_analysis`` and saved under
%   ``results/task7/<run_id>/``.
%
%   Usage:
%       metrics = Task_7('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat', ...
%                        'STATE_X001.txt');

if nargin < 2
    error('Task_7:BadArgs', 'Expected FUSED_FILE and TRUTH_FILE');
end

results_dir = get_results_dir();
[~, name, ~] = fileparts(fused_file);
run_id = regexprep(name, '_kf_output.*$', '');
out_dir = fullfile(results_dir, 'task7', run_id);
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

start_time = tic;

try
    task7_ned_residuals_plot(fused_file, truth_file, run_id, results_dir);
catch ME
    warning('NED residual plot failed: %s', ME.message);
end

try
    task7_ecef_residuals_plot(fused_file, '', '', truth_file, run_id, results_dir);
catch ME
    warning('ECEF residual plot failed: %s', ME.message);
end

metrics = task7_fused_truth_error_analysis(fused_file, truth_file, out_dir);
metrics_file = fullfile(out_dir, sprintf('%s_task7_metrics.mat', run_id));
save(metrics_file, 'metrics');

rows = {
    'Position [m]',    metrics.final_pos, metrics.rmse_pos;
    'Velocity [m/s]',  metrics.final_vel, metrics.rmse_vel;
    'Acceleration [m/s^2]', metrics.final_acc, metrics.rmse_acc};
header = {'Metric','FinalError','RMSE'};
T = cell2table(rows,'VariableNames',header);
disp(T);

runtime = toc(start_time);
fprintf('Task 7 runtime: %.2f s\n', runtime);
end
