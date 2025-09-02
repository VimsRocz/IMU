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
cfg.plots.save_pdf      = false;     % save as PDF
cfg.plots.save_png      = false;     % save as PNG

cfg.strict = true;   % tasks should error on missing inputs

% ZUPT detection thresholds (parity with Python defaults)
cfg.zupt_acc_var  = 0.01;   % accelerometer variance threshold
cfg.zupt_gyro_var = 1e-6;   % gyroscope variance threshold

% Kalman filter tuning (parity with Python defaults)
cfg.vel_q_scale   = 10.0; % scales Q(4:6,4:6) base (0.01)
cfg.vel_sigma_mps = 5.0;  % FIX: velocity measurement sigma [m/s]

% Gravity aiding (Task-5 EKF)
cfg.gaid = struct();
cfg.gaid.enabled       = true;   % enable gravity-direction aiding
cfg.gaid.fc_hz         = 0.4;    % LPF cutoff for accel [Hz]
cfg.gaid.sigma_dir_deg = 3.0;    % direction measurement noise per axis [deg]
cfg.gaid.max_dev_g     = 0.15;   % gate if | |a_lp|-g |/g > this
cfg.gaid.max_gyro_dps  = 5.0;    % gate if any |gyro| > this [deg/s]
cfg.gaid.min_speed_mps = 0.0;    % optional min speed for aiding [m/s]

% Yaw (course-over-ground) aiding
cfg.yawaid = struct();
cfg.yawaid.enabled        = true;  % enable GNSS course yaw aiding
cfg.yawaid.min_speed_mps  = 0.02;  % FIX: allow low-speed yaw aiding
cfg.yawaid.sigma_yaw_deg  = 5.0;   % yaw measurement noise [deg]
cfg.yawaid.max_gyro_dps   = 15.0;  % gate out fast turns [deg/s]
cfg.yawaid.gate_deg       = 25.0;  % residual gate for yaw innovation [deg]
% Lever arm (IMU->GNSS antenna) in body frame [m]
cfg.lever_arm = struct();
cfg.lever_arm.enabled = false;
cfg.lever_arm.r_b     = [0;0;0];
% Time alignment for measurements (Task-5)
cfg.time_align = struct();
cfg.time_align.apply = false;   % if true, shift GNSS by dt_s
cfg.time_align.dt_s  = 0.0;     % default shift [s] if no auto-estimate file
end
