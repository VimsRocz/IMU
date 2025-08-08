function run_triad_only(cfg)
%RUN_TRIAD_ONLY Execute Tasks 1–7 for a dataset using the TRIAD method.
%   RUN_TRIAD_ONLY(CFG) runs Tasks 1 through 7 unconditionally. A timeline
%   summary is printed first and written to ``MATLAB/results``. Truth data is
%   resolved automatically from a canonical path.

    % Ensure utils on path
    here = fileparts(mfilename('fullpath'));
    project_root = fileparts(here);
    utils_dir = fullfile(project_root,'src','utils');
    if exist(utils_dir,'dir'), addpath(utils_dir); end

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
    preferred_truth = fullfile(cfg.paths.root, 'STATE_IMU_X001.txt');
    fallback_truth  = fullfile(cfg.paths.root, 'STATE_X001.txt');
    if ~isfile(preferred_truth) && isfile(fallback_truth)
        copyfile(fallback_truth, preferred_truth);
        fprintf('Truth missing at preferred path; copying %s -> %s\n', fallback_truth, preferred_truth);
    end
    cfg.truth_path = preferred_truth;
    fprintf('Using TRUTH: %s\n', cfg.truth_path);

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
    t4_mat = fullfile(cfg.paths.matlab_results, ...
        sprintf('%s_%s_%s_task4_results.mat', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
    if ~isfile(t4_mat)
        error('Task 4 results was not created: %s', t4_mat);
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

