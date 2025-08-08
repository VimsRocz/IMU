function run_triad_only(cfg)
% RUN_TRIAD_ONLY — robust Tasks 1..7 runner (MATLAB)
% Prints timeline first, resolves TRUTH file, standardizes run_id & outputs,
% guarantees Task 4 produces a .mat (even if stub), and forces Task 6/7.

    % Ensure utils on path
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here,'src','utils'));

    % Config ----------------------------------------------------------------
    if nargin==0 || isempty(cfg)
        cfg = default_cfg();  % explicit starting values, visible here
        cfg.dataset_id = 'X002';
        cfg.method     = 'TRIAD';
        cfg.imu_file   = 'IMU_X002.dat';
        cfg.gnss_file  = 'GNSS_X002.csv';
        cfg.truth_file = '';  % use resolver (we always prefer STATE_IMU_X001.txt)
    end

    % Paths (MATLAB results separate from Python)
    cfg.paths = project_paths();
    results_dir = cfg.paths.matlab_results;
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    % Resolve absolute input paths
    cfg.imu_path  = fullfile(cfg.paths.root, cfg.imu_file);
    cfg.gnss_path = fullfile(cfg.paths.root, cfg.gnss_file);
    mustExist(cfg.imu_path,  'IMU file');
    mustExist(cfg.gnss_path, 'GNSS file');

    % Resolve TRUTH (always prefer STATE_IMU_X001.txt, copy from STATE_X001.txt)
    preferred_truth = '/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt';
    cfg.truth_path  = resolve_truth(preferred_truth);

    % Build run id and print timeline ---------------------------------------
    rid = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);
    print_timeline_matlab(rid, cfg.imu_path, cfg.gnss_path, cfg.truth_path, results_dir);

    fprintf('▶ %s\n', rid);
    fprintf('MATLAB results dir: %s\n', results_dir);

    % --- Tasks 1..5 ---------------------------------------------------------
    Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
    Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);

    % Task 4 must save "<rid>_task4_results.mat" — we also alias a legacy name.
    Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
    t4_main   = fullfile(results_dir, sprintf('%s_task4_results.mat', rid));
    t4_legacy = fullfile(results_dir, sprintf('Task4_results_%s.mat', strrep(rid,'_TRIAD','')));
    ensureTaskMat(t4_main, t4_legacy, rid);

    Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);

    % --- Tasks 6..7 mandatory-but-graceful ---------------------------------
    Task_6_wrapper(rid, cfg);  % will run / warn if truth missing
    Task_7_wrapper(rid, cfg);

end

% ----- wrappers to avoid hard fails if truth unavailable -------------------
function Task_6_wrapper(rid, cfg)
    try
        t5 = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_results.mat', rid));
        if ~isfile(t5)
            % common alternate name used by some code paths
            t5_alt = fullfile(cfg.paths.matlab_results, sprintf('%s_task5_results_%s.mat', rid, upper(string('TRIAD'))));
            if isfile(t5_alt), t5 = t5_alt; end
        end
        Task_6(t5, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    catch ME
        warning('Task 6 partial/skip: %s', ME.message);
    end
end

function Task_7_wrapper(~, ~)
    try
        Task_7();  % assumes it finds inputs under MATLAB/results
    catch ME
        warning('Task 7 partial/skip: %s', ME.message);
    end
end

% ----- helpers -------------------------------------------------------------
function mustExist(p, label)
    if ~isfile(p), error('%s not found: %s', label, p); end
end

function ensureTaskMat(t4_main, t4_legacy, rid)
    % Guarantee that Task 4 wrote something usable for downstream steps.
    if ~isfile(t4_main)
        warning('Task 4 output missing; creating minimal stub: %s', t4_main);
        imu_pos_g = []; imu_vel_g = []; imu_acc_g = []; t_g = []; %#ok<NASGU>
        save(t4_main,'rid','imu_pos_g','imu_vel_g','imu_acc_g','t_g','-v7');
    end
    % Also drop a legacy alias if needed by older code
    if ~isempty(t4_legacy) && ~isfile(t4_legacy)
        try
            copyfile(t4_main, t4_legacy);
        catch, end
    end
end
