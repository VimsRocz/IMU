function cfg = default_cfg()
% DEFAULT_CFG: starting config with no hidden defaults in tasks
cfg = struct();
cfg.dataset_id = 'X002';
cfg.method     = 'TRIAD';
cfg.imu_file   = 'IMU_X002.dat';
cfg.gnss_file  = 'GNSS_X002.csv';
cfg.truth_file = '';  % optional; set to 'STATE_X001.txt' if available

% plotting/runtime knobs (explicit, not hidden in tasks)
cfg.plots.popup = true;      % show figures interactively
cfg.plots.save  = true;      % also save PDFs/PNGs
cfg.strict = true;           % tasks should error on missing inputs
end
