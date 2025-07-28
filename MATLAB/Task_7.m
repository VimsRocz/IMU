function summary = Task_7(task5_file, truth_file)
%TASK_7 Residual analysis and summary after Task 5.
%   SUMMARY = TASK_7(TASK5_FILE, TRUTH_FILE) loads the fused trajectory
%   produced in Task 5 together with the ground truth state file and runs
%   the residual evaluation. All figures and metrics are saved to the
%   standard results directory under a dataset specific subfolder.
%
%   This helper mirrors the Python ``run_evaluation_npz`` workflow so that
%   the full MATLAB pipeline executes Tasks 1→7 automatically.
%
%   Example:
%       Task_7('results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat', ...
%              'STATE_X001.txt');

    if nargin < 2
        error('Task_7:BadArgs', 'Expected TASK5_FILE and TRUTH_FILE');
    end

    if ~isfile(task5_file)
        warning('Task 7 skipped: missing Task 5 file %s', task5_file);
        summary = struct();
        return;
    end
    if ~isfile(truth_file)
        warning('Task 7 skipped: missing truth file %s', truth_file);
        summary = struct();
        return;
    end

    [~, name, ~] = fileparts(task5_file);
    m = regexp(name, '(IMU_[^_]+)_(GNSS_[^_]+)_([A-Za-z]+)_task5_results', ...
                'tokens', 'once');
    if isempty(m)
        imu_name = 'IMU';
        gnss_name = 'GNSS';
        method = 'TRIAD';
    else
        imu_name = m{1};
        gnss_name = m{2};
        method   = m{3};
    end

    run_id = sprintf('%s_%s_%s', imu_name, gnss_name, method);
    results_dir = get_results_dir();
    out_dir = fullfile(results_dir, run_id);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    fprintf('--- Running Task 7 residual analysis for %s ---\n', run_id);
    try
        summary = task7_fused_truth_error_analysis(task5_file, truth_file, out_dir);
    catch ME
        fprintf('Task 7 error: %s\n', ME.message);
        summary = struct();
    end
    if ~isempty(fieldnames(summary))
        save_task_results(summary, imu_name, gnss_name, method, 7);
    end

    %% Additional simple residual analysis using downsampled x\_log
    task4_file = fullfile(results_dir, sprintf('Task4_results_%s_%s.mat', imu_name, gnss_name));
    if isfile(task4_file)
        try
            task7_basic_residuals_plot(task5_file, task4_file, run_id, out_dir);
        catch ME
            fprintf('Task 7 basic residuals failed: %s\n', ME.message);
        end
    else
        warning('Task 7 basic residuals skipped: missing %s', task4_file);
    end

    fprintf('Task 7 complete. See %s for plots and metrics.\n', out_dir);
end
