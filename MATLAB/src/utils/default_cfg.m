function cfg = default_cfg()
% DEFAULT_CFG: starting config with no hidden defaults in tasks
cfg = struct();
cfg.dataset_id = 'X002';
cfg.method     = 'TRIAD';
cfg.imu_file   = 'IMU_X002.dat';
cfg.gnss_file  = 'GNSS_X002.csv';
cfg.truth_file = '';

cfg.plots = struct();
cfg.plots.popup_figures = false;     % show figures by default?
cfg.plots.save_pdf      = true;      % save as PDF
cfg.plots.save_png      = true;      % save as PNG

cfg.strict = true;   % tasks should error on missing inputs
end
