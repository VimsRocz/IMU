function run_triad_only(cfg)
%RUN_TRIAD_ONLY Execute Tasks 1--7 for a dataset using the TRIAD method.
%   RUN_TRIAD_ONLY(CFG) processes IMU and GNSS inputs, summarises their
%   timelines and runs Tasks 1 through 7 sequentially.  Results are written
%   under ``MATLAB/results``.  CFG is an optional struct allowing fields
%   ``dataset_id``, ``method``, ``imu_file``, ``gnss_file`` and ``truth_file``.
%
%   Usage:
%       run_triad_only();
%       run_triad_only(struct('imu_file','IMU_X003.dat'));

% --- paths / utils --------------------------------------------------------
paths = project_paths();  % adds utils, sets root + MATLAB/results

% --- config ---------------------------------------------------------------
if nargin==0 || isempty(cfg), cfg = default_cfg(); end
if ~isfield(cfg,'dataset_id'), cfg.dataset_id = 'X002'; end
if ~isfield(cfg,'method'),     cfg.method     = 'TRIAD'; end
if ~isfield(cfg,'imu_file'),   cfg.imu_file   = 'IMU_X002.dat'; end
if ~isfield(cfg,'gnss_file'),  cfg.gnss_file  = 'GNSS_X002.csv'; end
if ~isfield(cfg,'truth_file'), cfg.truth_file = 'STATE_IMU_X001.txt'; end  % single truth file policy

cfg.paths = paths;

% resolve inputs (also copy into root if found elsewhere)
cfg.imu_path   = ensure_input_file('IMU',   cfg.imu_file,   cfg.paths);
cfg.gnss_path  = ensure_input_file('GNSS',  cfg.gnss_file,  cfg.paths);
cfg.truth_path = ensure_input_file('TRUTH', cfg.truth_file, cfg.paths);

% run id + timeline summary (MATLAB-only)
rid = run_id(cfg.imu_path, cfg.gnss_path, cfg.method);
print_timeline_matlab(rid, cfg.imu_path, cfg.gnss_path, cfg.truth_path, cfg.paths.matlab_results);

fprintf('▶ %s\n', rid);
fprintf('MATLAB results dir: %s\n', cfg.paths.matlab_results);

% --- Tasks 1..7 -----------------------------------------------------------
Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);
Task_6([], cfg.imu_path, cfg.gnss_path, cfg.truth_path); % your Task_6 signature may differ
Task_7();  % assumes it reads from MATLAB/results

end

