
%
% Usage (legacy “just run”):
%   run_triad_only   % uses default_cfg(), then you EDIT the struct below.
%
% NOTE: No hidden defaults are used inside tasks. All inputs come from cfg.

% --- Edit this block instead of hard-coding inside tasks ---
cfg = default_cfg();

% REQUIRED: declare exactly (no hidden defaults)
cfg.dataset_id = 'X002';
cfg.method     = 'TRIAD';
cfg.imu_file   = 'IMU_X002.dat';
cfg.gnss_file  = 'GNSS_X002.csv';
cfg.truth_file = 'STATE_X001.txt';     % leave as '' if not available



% Consistent naming/paths
cfg.paths = project_paths();            % root, results, src/utils added to path

% Resolve absolute input paths
cfg.imu_path  = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path = fullfile(cfg.paths.root, cfg.gnss_file);
if ~isempty(cfg.truth_file)
    cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);
else

