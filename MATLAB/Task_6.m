function Task_6(task5_file, imu_path, gnss_path, truth_file)
%TASK_6 Overlay ground truth on Task 5 results.
%   TASK_6(TASK5_FILE, IMU_PATH, GNSS_PATH, TRUTH_FILE) loads the
%   Task 5 results MAT file along with the raw IMU, GNSS and ground truth
%   trajectories. All series are interpolated to the estimator time
%   vector and ``plot_overlay`` is called for the NED, ECEF and body
%   frames. Truth data in the ECEF frame is first converted to the
%   estimator's local NED coordinates using ``compute_C_ECEF_to_NED`` so
%   that residuals are expressed in a consistent frame. The resulting
%   ``*_overlay_truth.pdf`` files are written to the directory returned by
%   ``project_paths()``. If TRUTH_FILE is empty it will be resolved using
%   ``resolve_truth_path``. This function expects the initialization
%   output from Task 1 and the filter output from Task 5 to reside in that
%   same directory.
%
% Usage:
%   Task_6(task5_file, imu_path, gnss_path, truth_file)

paths = project_paths();
results_dir = paths.matlab_results;
addpath(fullfile(paths.root,'MATLAB','lib'));
addpath(fullfile(paths.root,'src','utils'));
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
cfg = default_cfg();
visibleFlag = 'off';
try
    if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
        visibleFlag = 'on';
    end
catch
end

if nargin < 4
    error('Task_6:BadArgs', 'Expected TASK5_FILE, IMU_PATH, GNSS_PATH, TRUTH_FILE');
end

% Sign convention for NED -> NEU conversion
% (mirrors ``task6_plot_truth.py`` in the Python code)
% Use a row vector so element-wise operations broadcast correctly
sign_ned = [1, 1, -1];

fprintf('Starting Task 6 overlay ...\n');
start_time = tic;

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);


% paths and results_dir already defined above


if ~isfile(task5_file)
    error('Task_6:FileNotFound', 'Task 5 result not found: %s', task5_file);
end
S = load(task5_file);
if ~isfield(S, 'x_log')
    warning('Task_6:MissingData', ...
        'x_log field missing in %s. Attempting reconstruction.', task5_file);
    try
        S = reconstruct_x_log(S);
    catch ME
        warning('Task_6:ReconstructFailed', ...
            'Failed to reconstruct x_log: %s. Overlay skipped.', ME.message);
        fprintf('Task 6 overlay skipped: %s\n', ME.message);
        return
    end
end
fprintf('Task 6: Loaded x_log, size: %dx%d\n', size(S.x_log));

% Determine method from filename or structure.  The Task 5 results are
% named either ``<IMU>_<GNSS>_<METHOD>_task5_results.mat`` or
% ``<tag>_task5_results_<METHOD>.mat``.  Extract the method name without
% picking up dataset substrings like ``X001``.
tok = regexp(task5_file, '(TRIAD|Davenport|SVD)', 'match', 'once');
if ~isempty(tok)
    method = tok;
elseif isfield(S,'method')
    method = S.method;
else
    method = 'TRIAD';
end

% Build output directory using method and dataset identifiers
rid = run_id(imu_path, gnss_path, method);
run_id = rid;
out_dir = fullfile(results_dir, run_id);
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% Load gravity vector from Task 1 initialisation
% Use explicit components to avoid any ambiguity in the filename
pair_tag = [imu_name '_' gnss_name];
task1_file = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', ...
    imu_name, gnss_name, method));
if ~isfile(task1_file)
    % Fallback to methodless file for backward compatibility
    alt_file = fullfile(results_dir, sprintf('Task1_init_%s_%s.mat', ...
        imu_name, gnss_name));
    if isfile(alt_file)
        task1_file = alt_file;
    end
end
if isfile(task1_file)
    init_data = load(task1_file);
    if isfield(init_data, 'g_NED')
        g_NED = init_data.g_NED;
    else
        g_NED = [0;0;constants.GRAVITY];
    end
else
    warning('Task 1 output not found: %s', task1_file);
    g_NED = [0;0;constants.GRAVITY];
end

if nargin < 4 || isempty(truth_file)
    truth_file = resolve_truth_path();
end

% Reference coordinates from estimator or defaults
if isfield(S,'ref_lat'); ref_lat = S.ref_lat; else; ref_lat = deg2rad(-32.026554); end
if isfield(S,'ref_lon'); ref_lon = S.ref_lon; else; ref_lon = deg2rad(133.455801); end
if isfield(S,'ref_r0');  ref_r0 = S.ref_r0; else; ref_r0 = zeros(3,1); end
C = R_ecef_to_ned(ref_lat, ref_lon);
C_N_E = C';

if ~isfile(truth_file)
    warning('Truth file %s not found; using GNSS as truth.', truth_file);
    t_truth = S.gnss_time;
    pos_truth_ned_i_raw = S.gnss_pos_ned;
    vel_truth_ned_i_raw = S.gnss_vel_ned;
    acc_truth_ned_i_raw = [zeros(1,3); diff(vel_truth_ned_i_raw)./diff(t_truth)];
    pos_truth_ned_i = centre(pos_truth_ned_i_raw .* sign_ned);
    vel_truth_ned_i = vel_truth_ned_i_raw .* sign_ned;
    acc_truth_ned_i = acc_truth_ned_i_raw .* sign_ned;
    pos_ned_raw = S.pos_ned;
    vel_ned_raw = S.vel_ned;
    acc_ned_raw = [zeros(1,3); diff(vel_ned_raw)./diff(t_est)];
    pos_ned = centre(pos_ned_raw .* sign_ned);
    vel_ned = vel_ned_raw .* sign_ned;
    acc_ned = [zeros(1,3); diff(vel_ned)./diff(t_est)];
    t_imu = zero_base_time(t_est);
    t_truth = zero_base_time(t_truth);
    plot_state_grid(t_imu, {pos_truth_ned_i, pos_ned}, {vel_truth_ned_i, vel_ned}, {acc_truth_ned_i, acc_ned}, ...
        'NED', [run_id '_' method '_Task6_TruthOverlay'], out_dir, {'Truth','Fused'});
    return;
end

% Support text-based STATE_X files in addition to MAT files
if endsWith(truth_file, '.txt')
    ts = read_truth_state(truth_file);
    truth_time = ts.t;
    truth_pos_ecef = (C_N_E * ts.pos_ned')' + ref_r0';
    truth_vel_ecef = (C_N_E * ts.vel_ned')';
else
    S_truth = load(truth_file);
    if isfield(S_truth,'truth_time')
        truth_time = S_truth.truth_time;
    else
        truth_time = [];
    end
    truth_pos_ecef = S_truth.truth_pos_ecef;
    truth_vel_ecef = S_truth.truth_vel_ecef;
end
has_truth_time = ~isempty(truth_time);

I = C * C';
assert(max(abs(I(:) - eye(3))) < 1e-9, 'R_ecef_to_ned not orthonormal');

% Ground truth arrays
t_truth = truth_time;
pos_truth_ecef = truth_pos_ecef;
vel_truth_ecef = truth_vel_ecef;
if has_truth_time
    acc_truth_ecef = [zeros(1,3); diff(vel_truth_ecef)./diff(t_truth)];
else
    acc_truth_ecef = [];
end

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
    if isfield(S,'fused_pos')
        S.pos_ned = S.fused_pos;
    else
        S.pos_ned = S.x_log(1:3,:)';
    end
end
if ~isfield(S,'vel_ned')
    if isfield(S,'fused_vel')
        S.vel_ned = S.fused_vel;
    else
        S.vel_ned = S.x_log(4:6,:)';
    end
end

dt_est = mean(diff(t_est));

if has_truth_time && numel(t_truth) == size(pos_truth_ecef,1)
    pos_truth_ecef_i = interp1(t_truth, pos_truth_ecef, t_est, 'linear', 'extrap');
    vel_truth_ecef_i = interp1(t_truth, vel_truth_ecef, t_est, 'linear', 'extrap');
    acc_truth_ecef_i = interp1(t_truth, acc_truth_ecef, t_est, 'linear', 'extrap');
else
    C_N_E = C';
    est_pos_ecef = (C_N_E*S.pos_ned')' + ref_r0';
    [lag, t_shift] = compute_time_shift(est_pos_ecef(:,1), truth_pos_ecef(:,1), dt_est);
    fprintf('Task 6: aligned truth by %d samples (%.3f s) using xcorr\n', lag, t_shift);
    if lag >= 0
        est_idx = 1:(size(est_pos_ecef,1)-lag);
        truth_idx = (1+lag):size(truth_pos_ecef,1);
    else
        lag = abs(lag);
        est_idx = (1+lag):size(est_pos_ecef,1);
        truth_idx = 1:(size(truth_pos_ecef,1)-lag);
    end
    n = min(numel(est_idx), numel(truth_idx));
    est_idx = est_idx(1:n);
    truth_idx = truth_idx(1:n);
    t_est = t_est(est_idx);
    S.pos_ned = S.pos_ned(est_idx,:);
    S.vel_ned = S.vel_ned(est_idx,:);
    pos_truth_ecef_i = truth_pos_ecef(truth_idx,:);
    vel_truth_ecef_i = truth_vel_ecef(truth_idx,:);
    acc_truth_ecef_i = [zeros(1,3); diff(vel_truth_ecef_i)./dt_est];
end

pos_truth_ned_i_raw  = (C * (pos_truth_ecef_i' - ref_r0)).';
vel_truth_ned_i_raw  = (C*vel_truth_ecef_i.').';
acc_truth_ned_i_raw  = (C*acc_truth_ecef_i.').';

pos_truth_ned_i = centre(pos_truth_ned_i_raw .* sign_ned);
vel_truth_ned_i = vel_truth_ned_i_raw .* sign_ned;
acc_truth_ned_i = acc_truth_ned_i_raw .* sign_ned;

% Fused IMU results and derived acceleration
pos_ned_raw = S.pos_ned;
vel_ned_raw = S.vel_ned;
acc_ned_raw = [zeros(1,3); diff(vel_ned_raw)./diff(t_est)];

pos_ned = centre(pos_ned_raw .* sign_ned);
vel_ned = vel_ned_raw .* sign_ned;
acc_ned = [zeros(1,3); diff(vel_ned)./diff(t_est)];

t_imu = zero_base_time(t_est);
plot_state_grid(t_imu, {pos_truth_ned_i, pos_ned}, {vel_truth_ned_i, vel_ned}, {acc_truth_ned_i, acc_ned}, ...
    'NED', [run_id '_' method '_Task6_TruthOverlay'], out_dir, {'Truth','Fused'});

% 3x3 overlay grid comparing fused vs truth
fused_struct = struct('t', t_est, 'pos', pos_ned, 'vel', vel_ned, 'acc', acc_ned);
truth_struct = struct('t', t_est, 'pos', pos_truth_ned_i, ...
                      'vel', vel_truth_ned_i, 'acc', acc_truth_ned_i);
if has_truth_time && ~isempty(t_truth)
    t_truth_ref = t_truth(1:numel(t_est));
    t_ref = zero_base_time(t_truth_ref);
    truth_struct.t = t_truth_ref;
else
    t_ref = t_imu;
end
plot_state_grid_overlay(t_ref, fused_struct, truth_struct, 'NED', ...
    'Title', sprintf('%s Task6: Fused vs Truth', run_id), ...
    'Visible', visibleFlag);
saveas(gcf, fullfile(out_dir, sprintf('%s_task6_overlay_grid_NED.png', run_id)));
saveas(gcf, fullfile(out_dir, sprintf('%s_task6_overlay_grid_NED.pdf', run_id)));

C_N_E = C';
pos_ecef = (C_N_E*pos_ned_raw')' + ref_r0';
vel_ecef = (C_N_E*vel_ned_raw')';
acc_ecef = (C_N_E*acc_ned_raw')';

cn = corrcoef(vel_ned(:,1), vel_truth_ned_i(:,1));  % North
ce = corrcoef(vel_ned(:,2), vel_truth_ned_i(:,2));  % East
cx = corrcoef(vel_ecef(:,1), vel_truth_ecef_i(:,1));% X
cy = corrcoef(vel_ecef(:,2), vel_truth_ecef_i(:,2));% Y
fprintf('[Sanity] corr(N)=%.3f  corr(E)=%.3f  corr(X)=%.3f  corr(Y)=%.3f\n', ...
        cn(1,2), ce(1,2), cx(1,2), cy(1,2));

% Body frame conversion
if ~exist('g_NED','var')
    g_NED = [0;0;constants.GRAVITY];
end
N = length(t_est);
pos_body = zeros(N,3); vel_body = zeros(N,3); acc_body = zeros(N,3);
pos_truth_body = zeros(N,3); vel_truth_body = zeros(N,3); acc_truth_body = zeros(N,3);
for k = 1:N
    C_B_N = euler_to_rot(S.euler_log(:,k));
    pos_body(k,:) = (C_B_N'*pos_ned_raw(k,:)')';
    vel_body(k,:) = (C_B_N'*vel_ned_raw(k,:)')';
    acc_body(k,:) = (C_B_N'*(acc_ned_raw(k,:)' - g_NED))';
    pos_truth_body(k,:) = (C_B_N'*pos_truth_ned_i_raw(k,:)')';
    vel_truth_body(k,:) = (C_B_N'*vel_truth_ned_i_raw(k,:)')';
    acc_truth_body(k,:) = (C_B_N'*(acc_truth_ned_i_raw(k,:)' - g_NED))';
end

save_overlay_state(t_est, pos_ned, vel_ned, pos_truth_ned_i, vel_truth_ned_i, ...
    'NED', run_id, method, out_dir);
save_overlay_state(t_est, pos_ecef, vel_ecef, pos_truth_ecef_i, vel_truth_ecef_i, ...
    'ECEF', run_id, method, out_dir);
save_overlay_state(t_est, pos_body, vel_body, pos_truth_body, vel_truth_body, ...
    'Body', run_id, method, out_dir);
% ------------------------------------------------------------------
% Compute overlay metrics for summary tables
% ------------------------------------------------------------------
[mNED, ~]  = compute_overlay_metrics(t_est, pos_ned,  vel_ned,  pos_truth_ned_i,  vel_truth_ned_i);
[mECEF, ~] = compute_overlay_metrics(t_est, pos_ecef, vel_ecef, pos_truth_ecef_i, vel_truth_ecef_i);
[mBody, ~] = compute_overlay_metrics(t_est, pos_body, vel_body, pos_truth_body,  vel_truth_body);
metrics = struct('NED', mNED, 'ECEF', mECEF, 'Body', mBody);
metrics_file = fullfile(out_dir, sprintf('%s_task6_metrics.mat', run_id));
save_overwrite(metrics_file, 'metrics');
rows = {
    'NED',  mNED.rmse_pos,  mNED.final_pos,  mNED.rmse_vel,  mNED.final_vel,  mNED.rmse_acc,  mNED.final_acc;
    'ECEF', mECEF.rmse_pos, mECEF.final_pos, mECEF.rmse_vel, mECEF.final_vel, mECEF.rmse_acc, mECEF.final_acc;
    'Body', mBody.rmse_pos, mBody.final_pos, mBody.rmse_vel, mBody.final_vel, mBody.rmse_acc, mBody.final_acc};
header = {'Frame','RMSEpos','FinalPos','RMSEvel','FinalVel','RMSEacc','FinalAcc'};
T = cell2table(rows,'VariableNames',header);
disp(T);
runtime = toc(start_time);
fprintf('Task 6 runtime: %.2f s\n', runtime);
fprintf('[SUMMARY] method=%s rmse_pos=%.2f m final_pos=%.2f m ', ...
        method, mNED.rmse_pos, mNED.final_pos);
fprintf('rmse_vel=%.2f m/s final_vel=%.2f m/s\n', ...
        mNED.rmse_vel, mNED.final_vel);
results = struct('metrics', metrics, 'runtime', runtime);
save_task_results(results, imu_name, gnss_name, method, 6);
end

function y = centre(x)
%CENTRE Remove the mean from each column vector.
    y = x - mean(x,1);
end
