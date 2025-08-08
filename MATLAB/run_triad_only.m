function run_triad_only(cfg)
%RUN_TRIAD_ONLY Execute Tasks 1–7 for a dataset using the TRIAD method.
%   RUN_TRIAD_ONLY(CFG) runs Tasks 1 through 7 unconditionally. A timeline
%   summary is printed first and written to ``MATLAB/results``. Truth data is
%   resolved automatically from a canonical path.

    if nargin==0 || isempty(cfg)
        cfg = default_cfg();
        cfg.dataset_id = 'X002';
        cfg.method     = 'TRIAD';
        cfg.imu_file   = 'IMU_X002.dat';
        cfg.gnss_file  = 'GNSS_X002.csv';
        cfg.truth_file = '';
    end
    cfg.paths = project_paths();
    results_dir = cfg.paths.matlab_results;
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    % Absolute paths
    cfg.imu_path  = fullfile(cfg.paths.root, cfg.imu_file);
    cfg.gnss_path = fullfile(cfg.paths.root, cfg.gnss_file);

    % Build run id
    rid = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);

    % Prefer a single canonical TRUTH file
    if ~isfield(cfg,'truth_path') || ~isfile(cfg.truth_path)
        src = fullfile(cfg.paths.root, 'STATE_X001.txt');
        dst = fullfile(cfg.paths.root, 'STATE_IMU_X001.txt');
        if isfile(src), copyfile(src, dst); end
        cfg.truth_path = dst;
    end
    fprintf('Using TRUTH: %s\n', cfg.truth_path);

    % Print timeline (robust implementation)
    print_timeline_matlab(rid, cfg.imu_path, cfg.gnss_path, cfg.truth_path, cfg.paths.matlab_results);

    fprintf('▶ %s\n', rid);
    fprintf('MATLAB results dir: %s\n', results_dir);

    % Tasks 1–5
    Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
    t4_mat_actual = fullfile(cfg.paths.matlab_results, sprintf('%s_task4_results.mat', rid));
    if ~isfile(t4_mat_actual)
        error('Task 4 results was not created: %s', t4_mat_actual);
    end
    Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);

    % Tasks 6–7 mandatory (graceful if no truth)
    Task_6_wrapper(rid, cfg);
    Task_7_wrapper(rid, cfg);
end

% -- wrappers to avoid hard fails if truth unavailable
function Task_6_wrapper(rid, cfg)
    try
        Task_6(fullfile(cfg.paths.matlab_results, sprintf('%s_task5_results.mat', rid)), ...
               cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    catch ME
        warning('Task 6 skipped or partial: %s', ME.message);
    end
end

function Task_7_wrapper(~, ~)
    try
        Task_7();
    catch ME
        warning('Task 7 skipped or partial: %s', ME.message);
    end
end

