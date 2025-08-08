function run_triad_only(cfg)
%RUN_TRIAD_ONLY Execute Tasks 1–7 for a dataset using the TRIAD method.
%   RUN_TRIAD_ONLY(CFG) runs Tasks 1 through 7 unconditionally. A timeline
%   summary is printed first and written to ``MATLAB/results``. Truth data is
%   resolved automatically from a canonical path.

    % Ensure utils on path
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here,'src','utils'));

    % Config
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

    % TRUTH: canonical preferred path; copy if missing
    preferred_truth = '/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt';
    cfg.truth_path  = resolve_truth(preferred_truth);

    % Build run_id and print timeline
    rid = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);
    print_timeline_matlab(rid, cfg.imu_path, cfg.gnss_path, cfg.truth_path, results_dir);

    fprintf('▶ %s\n', rid);
    fprintf('MATLAB results dir: %s\n', results_dir);

    % Tasks 1–5
    Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
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

