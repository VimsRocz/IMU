function run_all_methods()
%RUN_ALL_METHODS Execute Tasks 1--7 for the X002 dataset using TRIAD.
%   This helper mirrors ``src/run_triad_only.py`` but uses the MATLAB
%   implementation exclusively.  All intermediate files are generated
%   and consumed by MATLAB so the pipeline runs independently of any
%   Python results. Dataset files are referenced directly from the
%   repository root and outputs are stored under ``results/`` using the
%   standard naming convention.

    dataset.imu  = 'IMU_X002.dat';
    dataset.gnss = 'GNSS_X002.csv';
    method = 'TRIAD';

    [~, imu_name, ~]  = fileparts(dataset.imu);
    [~, gnss_name, ~] = fileparts(dataset.gnss);
    run_id = sprintf('%s_%s_%s', imu_name, gnss_name, method);

    root_dir  = fileparts(fileparts(mfilename('fullpath')));
    imu_path  = fullfile(root_dir, dataset.imu);
    gnss_path = fullfile(root_dir, dataset.gnss);

    if ~isfile(imu_path) || ~isfile(gnss_path)
        error('File not found: %s or %s', imu_path, gnss_path);
    end

    Task_1(imu_path, gnss_path, method);

    Task_2(imu_path, gnss_path, method);
    Task_3(imu_path, gnss_path, method);
    Task_4(imu_path, gnss_path, method);
    Task_5(imu_path, gnss_path, method);

    res_file = fullfile(get_results_dir(), sprintf('IMU_X002_GNSS_X002_%s_task5_results.mat', method));
    truth_file = fullfile(root_dir, 'STATE_X001.txt');
    if isfile(res_file) && isfile(truth_file)
        disp('--- Running Task 6: Truth Overlay/Validation ---');
        Task_6(res_file, imu_path, gnss_path, truth_file);
        disp('--- Running Task 7: Residuals & Summary ---');
        Task_7(res_file, truth_file, run_id);
        disp('Task 6 and Task 7 complete. See results directory for plots and PDF summaries.');
    else
        warning('Task 6 or Task 7 skipped: Missing Task 5 results or truth file.');
    end
end
