%RUN_TRIAD_ONLY  Process dataset with TRIAD method via a structured driver.
% Usage (recommended):
%   cfg = default_cfg();           % edit fields explicitly below if needed
%   cfg.dataset_id = 'X002';       % REQUIRED: dataset tag
%   cfg.method     = 'TRIAD';      % REQUIRED: 'TRIAD' (others ok if implemented)
%   cfg.imu_file   = 'IMU_X002.dat';
%   cfg.gnss_file  = 'GNSS_X002.csv';
%   cfg.truth_file = 'STATE_X001.txt';  % optional for Task 6/7
%   run_triad(cfg);
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

% Plotting policy (pop-up & save)
cfg.plots.popup_figures = true;        % show MATLAB figures
cfg.plots.save_pdf      = true;
cfg.plots.save_png      = true;

% Consistent naming/paths
cfg.paths = project_paths();            % root, results, src/utils added to path

% Resolve absolute input paths
cfg.imu_path  = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path = fullfile(cfg.paths.root, cfg.gnss_file);
if ~isempty(cfg.truth_file)
    cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);
else
    cfg.truth_path = '';
end

% Run
run_triad(cfg);
