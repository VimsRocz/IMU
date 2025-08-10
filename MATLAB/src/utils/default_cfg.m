function cfg = default_cfg()
% DEFAULT_CFG: starting config with no hidden defaults in tasks
cfg = struct();
cfg.dataset_id = 'X002';
cfg.method     = 'TRIAD';
cfg.imu_file   = 'IMU_X002.dat';
cfg.gnss_file  = 'GNSS_X002.csv';
cfg.truth_file = '';

cfg.plots = struct();
cfg.plots.popup_figures = true;      % pop up figures by default
cfg.plots.save_pdf      = true;      % save as PDF
cfg.plots.save_png      = true;      % save as PNG

cfg.strict = true;   % tasks should error on missing inputs

% ZUPT detection thresholds (parity with Python defaults)
cfg.zupt_acc_var  = 0.01;   % accelerometer variance threshold
cfg.zupt_gyro_var = 1e-6;   % gyroscope variance threshold

% Kalman filter tuning (parity with Python defaults)
cfg.vel_q_scale = 10.0;   % scales Q(4:6,4:6) base (0.01)
cfg.vel_r       = 0.25;   % R(4:6,4:6) diagonal [m^2/s^2]
end
