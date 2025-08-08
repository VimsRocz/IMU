function run_triad_only(cfg)
%RUN_TRIAD_ONLY  Process dataset using TRIAD (Tasks 1..7) — MATLAB-only results.
%   run_triad_only(cfg) executes the MATLAB pipeline using the configuration
%   struct ``cfg``.  When ``cfg`` is omitted, :func:`default_cfg` provides a
%   starting configuration with no hidden defaults.

if nargin==0 || isempty(cfg)
    cfg = default_cfg();
    cfg.dataset_id = 'X002';
    cfg.method     = 'TRIAD';
    cfg.imu_file   = 'IMU_X002.dat';
    cfg.gnss_file  = 'GNSS_X002.csv';
    cfg.truth_file = 'STATE_X001.txt';  % set '' if unavailable
end

cfg.paths = project_paths();  % adds MATLAB/src + utils to path
cfg.imu_path  = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path = fullfile(cfg.paths.root, cfg.gnss_file);
if ~isempty(cfg.truth_file)
    cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);
else
    cfg.truth_path = '';
end

assert(isfile(cfg.imu_path),  'IMU file missing: %s',  cfg.imu_path);
assert(isfile(cfg.gnss_path), 'GNSS file missing: %s', cfg.gnss_path);

results_dir = cfg.paths.matlab_results;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

fprintf('MATLAB results dir: %s\n', results_dir);
fprintf('> %s_%s_%s\n', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method);

Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);

t5 = fullfile(results_dir, sprintf('%s_%s_%s_task5_results.mat', ...
     erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));

if isfile(t5) && ~isempty(cfg.truth_path) && isfile(cfg.truth_path)
    disp('--- Running Task 6: Truth Overlay/Validation ---');
    Task_6(t5, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    run_id = sprintf('%s_%s_%s', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method);
    out_dir = fullfile(results_dir, run_id);
    fprintf('Task 6 overlay plots saved under: %s\n', out_dir);

    disp('--- Running Task 7: Residuals & Summary ---');
    Task_7();  % reads its inputs from MATLAB/results
    fprintf('Task 7 evaluation plots saved under: %s\n', out_dir);
else
    warning('Task 6/7 skipped: Missing Task 5 results or truth file.');
end
end
