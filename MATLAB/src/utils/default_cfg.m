function cfg = default_cfg()
% DEFAULT_CFG: starting config with no hidden defaults in tasks
cfg = struct();
cfg.dataset_id = 'X002';
cfg.method     = 'TRIAD';
cfg.imu_file   = 'IMU_X002.dat';
cfg.gnss_file  = 'GNSS_X002.csv';
cfg.truth_file = '';  % optional; set 'STATE_X001.txt' if you actually have it

cfg.plots.popup = true;   % show figures
cfg.plots.save  = true;   % and save to disk
cfg.strict      = true;   % tasks should error on missing inputs
end
