function Task_6(task5_file, imu_path, gnss_path, truth_file)
%TASK_6 Overlay ground truth on Task 5 results.
%   TASK_6(TASK5_FILE, IMU_PATH, GNSS_PATH, TRUTH_FILE) loads the
%   Task 5 results MAT file along with the raw IMU, GNSS and ground truth
%   trajectories.  All series are interpolated to the estimator time
%   vector and ``plot_overlay`` is called for the NED, ECEF and body
%   frames.  The resulting ``*_overlay_truth.pdf`` files are stored under
%   ``results/``.

if nargin < 4
    error('Task_6:BadArgs', 'Expected TASK5_FILE, IMU_PATH, GNSS_PATH, TRUTH_FILE');
end

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
results_dir = fullfile(root, 'results');

if ~isfile(task5_file)
    error('Task_6:FileNotFound', 'Task 5 result not found: %s', task5_file);
end
S = load(task5_file);

% Derive method tag from filename or structure
tok = regexp(task5_file, '_(\w+)_task5_results\.mat$', 'tokens');
if ~isempty(tok)
    method = tok{1}{1};
elseif isfield(S,'method')
    method = S.method;
else
    method = 'TRIAD';
end

if nargin < 4 || isempty(truth_file)
    % Default to the common STATE_X001.txt trajectory
    state_name = 'STATE_X001.txt';
    try
        truth_file = get_data_file(state_name);
    catch
        error('Task_6:TruthMissing', 'State file not found: %s', state_name);
    end
end

% Load STATE_X truth file with comment support
truth = read_state_file(truth_file);

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
elseif isfield(S,'time')
    t_est = S.time;
elseif isfield(S,'imu_time')
    t_est = S.imu_time;
else
    t_est = (0:size(S.x_log,2)-1)';
end

if ~isfield(S, 'gnss_time')
    S.gnss_time = linspace(t_est(1), t_est(end), size(S.gnss_pos_ned,1))';
end

if ~isfield(S,'pos_ned')
    S.pos_ned = S.x_log(1:3,:)';
end
if ~isfield(S,'vel_ned')
    S.vel_ned = S.x_log(4:6,:)';
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

% ------------------------------------------------------------------
% Compute overlay metrics for summary tables
% ------------------------------------------------------------------
[mNED, ~]  = compute_overlay_metrics(t_est, pos_ned,  vel_ned,  pos_truth_ned_i,  vel_truth_ned_i);
[mECEF, ~] = compute_overlay_metrics(t_est, pos_ecef, vel_ecef, pos_truth_ecef_i, vel_truth_ecef_i);
[mBody, ~] = compute_overlay_metrics(t_est, pos_body, vel_body, pos_truth_body,  vel_truth_body);
metrics = struct('NED', mNED, 'ECEF', mECEF, 'Body', mBody);
metrics_file = fullfile(results_dir, sprintf('%s_%s_%s_task6_metrics.mat', ...
    imu_name, gnss_name, method));
save(metrics_file, 'metrics');
fprintf('[Task6] %s %s RMSEpos(NED)=%.3f m final=%.3f m\n', ...
    imu_name, method, mNED.rmse_pos, mNED.final_pos);
end

