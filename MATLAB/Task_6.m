function Task_6(imu_path, gnss_path, method)
%TASK_6 Overlay ground truth on Task 5 results.
%   TASK_6(IMU_PATH, GNSS_PATH, METHOD) loads the
%   <IMU>_<GNSS>_<METHOD>_kf_output.mat file produced by Task 5 and the
%   matching STATE_*.txt trajectory. IMU, GNSS and truth data are
%   interpolated to a common time vector and passed to PLOT_OVERLAY for
%   the NED, ECEF and body frames. The resulting PDFs
%   <METHOD>_<FRAME>_overlay_truth.pdf are stored in the results folder.

if nargin < 3 || isempty(method)
    method = 'TRIAD';
end

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
results_dir = 'results';

kf_file = fullfile(results_dir, sprintf('%s_%s_%s_kf_output.mat', ...
    imu_name, gnss_name, method));
if ~isfile(kf_file)
    error('Task_6:FileNotFound', 'KF output not found: %s', kf_file);
end

state_name = [strrep(imu_name,'IMU','STATE') '.txt'];
try
    state_file = get_data_file(state_name);
catch
    error('Task_6:TruthMissing', 'State file not found: %s', state_name);
end

S = load(kf_file);
truth = load(state_file);

% Use reference coordinates from the estimate when available
if isfield(S,'ref_lat'); ref_lat = S.ref_lat; else; ref_lat = deg2rad(-32.026554); end
if isfield(S,'ref_lon'); ref_lon = S.ref_lon; else; ref_lon = deg2rad(133.455801); end
if isfield(S,'ref_r0');  ref_r0 = S.ref_r0;  else;  ref_r0 = truth(1,3:5)'; end
C = compute_C_ECEF_to_NED(ref_lat, ref_lon);

% Ground truth in different frames
t_truth = truth(:,2);
pos_truth_ecef = truth(:,3:5);
vel_truth_ecef = truth(:,6:8);
acc_truth_ecef = [zeros(1,3); diff(vel_truth_ecef)./diff(t_truth)];
pos_truth_ned = (C*(pos_truth_ecef' - ref_r0(:)))';
vel_truth_ned = (C*vel_truth_ecef')';
acc_truth_ned = (C*acc_truth_ecef')';

% Time vector from estimator
if isfield(S,'time_residuals') && ~isempty(S.time_residuals)
    t_est = S.time_residuals;
else
    t_est = S.time;
end

% Interpolate truth and GNSS to estimator time
pos_truth_ned_i  = interp1(t_truth, pos_truth_ned,  t_est);
vel_truth_ned_i  = interp1(t_truth, vel_truth_ned,  t_est);
acc_truth_ned_i  = interp1(t_truth, acc_truth_ned,  t_est);
pos_truth_ecef_i = interp1(t_truth, pos_truth_ecef, t_est);
vel_truth_ecef_i = interp1(t_truth, vel_truth_ecef, t_est);
acc_truth_ecef_i = interp1(t_truth, acc_truth_ecef, t_est);

pos_gnss_ned_i  = interp1(S.gnss_time, S.gnss_pos_ned,  t_est);
vel_gnss_ned_i  = interp1(S.gnss_time, S.gnss_vel_ned,  t_est);
acc_gnss_ned_i  = interp1(S.gnss_time, S.gnss_accel_ned, t_est);

pos_gnss_ecef_i = interp1(S.gnss_time, S.gnss_pos_ecef,  t_est);
vel_gnss_ecef_i = interp1(S.gnss_time, S.gnss_vel_ecef,  t_est);
acc_gnss_ecef_i = interp1(S.gnss_time, S.gnss_accel_ecef, t_est);

% Fused IMU results and derived acceleration
pos_ned = S.pos_ned;
vel_ned = S.vel_ned;
acc_ned = [zeros(1,3); diff(vel_ned)./diff(t_est)];

C_N_E = C';
pos_ecef = (C_N_E*pos_ned')' + ref_r0';
vel_ecef = (C_N_E*vel_ned')';
acc_ecef = (C_N_E*acc_ned')';

% Body frame conversion
g_N = [0 0 constants.GRAVITY]';
N = length(t_est);
pos_body = zeros(N,3); vel_body = zeros(N,3); acc_body = zeros(N,3);
pos_gnss_body = zeros(N,3); vel_gnss_body = zeros(N,3); acc_gnss_body = zeros(N,3);
pos_truth_body = zeros(N,3); vel_truth_body = zeros(N,3); acc_truth_body = zeros(N,3);
for k = 1:N
    C_B_N = euler_to_rot(S.euler_log(:,k));
    pos_body(k,:) = (C_B_N'*pos_ned(k,:)')';
    vel_body(k,:) = (C_B_N'*vel_ned(k,:)')';
    acc_body(k,:) = (C_B_N'*(acc_ned(k,:)' - g_N))';
    pos_gnss_body(k,:) = (C_B_N'*pos_gnss_ned_i(k,:)')';
    vel_gnss_body(k,:) = (C_B_N'*vel_gnss_ned_i(k,:)')';
    acc_gnss_body(k,:) = (C_B_N'*(acc_gnss_ned_i(k,:)' - g_N))';
    pos_truth_body(k,:) = (C_B_N'*pos_truth_ned_i(k,:)')';
    vel_truth_body(k,:) = (C_B_N'*vel_truth_ned_i(k,:)')';
    acc_truth_body(k,:) = (C_B_N'*(acc_truth_ned_i(k,:)' - g_N))';
end

plot_overlay('NED', method, t_est, pos_ned, vel_ned, acc_ned, ...
    t_est, pos_gnss_ned_i, vel_gnss_ned_i, acc_gnss_ned_i, ...
    t_est, pos_ned, vel_ned, acc_ned, results_dir, ...
    't_truth', t_est, 'pos_truth', pos_truth_ned_i, ...
    'vel_truth', vel_truth_ned_i, 'acc_truth', acc_truth_ned_i, ...
    'suffix', '_overlay_truth.pdf');

plot_overlay('ECEF', method, t_est, pos_ecef, vel_ecef, acc_ecef, ...
    t_est, pos_gnss_ecef_i, vel_gnss_ecef_i, acc_gnss_ecef_i, ...
    t_est, pos_ecef, vel_ecef, acc_ecef, results_dir, ...
    't_truth', t_est, 'pos_truth', pos_truth_ecef_i, ...
    'vel_truth', vel_truth_ecef_i, 'acc_truth', acc_truth_ecef_i, ...
    'suffix', '_overlay_truth.pdf');

plot_overlay('Body', method, t_est, pos_body, vel_body, acc_body, ...
    t_est, pos_gnss_body, vel_gnss_body, acc_gnss_body, ...
    t_est, pos_body, vel_body, acc_body, results_dir, ...
    't_truth', t_est, 'pos_truth', pos_truth_body, ...
    'vel_truth', vel_truth_body, 'acc_truth', acc_truth_body, ...
    'suffix', '_overlay_truth.pdf');
end
