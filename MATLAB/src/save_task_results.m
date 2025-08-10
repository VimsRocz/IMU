function save_task_results(results, imu_name, gnss_name, method, task)
%SAVE_TASK_RESULTS Save a struct to the results directory.
%   SAVE_TASK_RESULTS(RESULTS, IMU_NAME, GNSS_NAME, METHOD, TASK) stores the
%   struct RESULTS as a MAT-file following the naming convention used by
%   the Python pipeline.
%
%   Example:
%       save_task_results(r, 'IMU_X002', 'GNSS_X002', 'TRIAD', 3)
%   produces results/IMU_X002_GNSS_X002_TRIAD_task3_results.mat

    results_dir = get_results_dir();
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    base = sprintf('%s_%s_%s_task%d_results.mat', imu_name, gnss_name, method, task);
    out  = fullfile(results_dir, base);

    % Append when the file already exists so previously saved variables
    % such as ``x_log`` are not overwritten. Earlier tasks create the file
    % anew, whereas Task 5 may save additional arrays before calling this
    % helper.  This mirrors the Python pipeline behaviour where multiple
    % keys coexist in the same ``.mat`` container.
    if isfile(out)
        save(out, 'results', '-append');
    else
        save(out, 'results');
    end
    fprintf('Saved task %d results to %s\n', task, out);
end
