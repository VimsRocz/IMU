function Task_6(task5_file, imu_path, gnss_path, truth_file)
%TASK_6 Overlay ground truth on Task 5 results.
%   TASK_6(TASK5_FILE, IMU_PATH, GNSS_PATH, TRUTH_FILE) loads the
%   Task 5 results MAT file along with the raw IMU, GNSS and ground truth
%   trajectories.  All series are interpolated to the estimator time
%   vector and ``plot_overlay`` is called for the NED, ECEF and body
%   frames.  Truth data in the ECEF frame is first converted to the
%   estimator's local NED coordinates using ``compute_C_ECEF_to_NED`` so
%   that residuals are expressed in a consistent frame.  The resulting
%   ``*_overlay_truth.pdf`` files are written to the directory returned by
%   ``get_results_dir()``.  This function expects the initialization output
%   from Task 1 and the filter output from Task 5 to reside in that same
%   directory.
%
% Usage:
%   Task_6(task5_file, imu_path, gnss_path, truth_file)

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

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
% Results directory under repository root
results_dir = get_results_dir();
if ~exist(results_dir, 'dir'); mkdir(results_dir); end


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

% Load GNSS truth data from Task 4
truth4_file = fullfile(results_dir, 'Task4_results_IMU_X002_GNSS_X002.mat');
try
    load(truth4_file, 'gnss_pos_ned');
    fprintf('Task 6: Loaded GNSS truth positions from %s, size: %dx%d\n', ...
        truth4_file, size(gnss_pos_ned));
catch
    error('Task 6: Failed to load GNSS truth data from %s.', truth4_file);
end

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
run_id = sprintf('%s_%s_%s', imu_name, gnss_name, method);
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
    truth_file = fullfile(results_dir, 'Task4_results_IMU_X002_GNSS_X002.mat');
end

% Support text-based STATE_X files in addition to MAT files
if endsWith(truth_file, '.txt')
    raw = read_state_file(truth_file);
    truth_pos_ecef = raw(:,3:5);
    truth_vel_ecef = raw(:,6:8);
else
    S_truth = load(truth_file, 'truth_pos_ecef', 'truth_vel_ecef');
    truth_pos_ecef = S_truth.truth_pos_ecef;
    truth_vel_ecef = S_truth.truth_vel_ecef;
end
truth_time = (0:size(truth_pos_ecef,1)-1)';
truth = [zeros(size(truth_time)), truth_time, truth_pos_ecef, truth_vel_ecef];

% Use reference coordinates from the estimate when available
if isfield(S,'ref_lat'); ref_lat = S.ref_lat; else; ref_lat = deg2rad(-32.026554); end
if isfield(S,'ref_lon'); ref_lon = S.ref_lon; else; ref_lon = deg2rad(133.455801); end
if isfield(S,'ref_r0');  ref_r0 = S.ref_r0;  else;  ref_r0 = truth(1,3:5)'; end
C = compute_C_ECEF_to_NED(ref_lat, ref_lon);
fprintf('[DBG-FRAME] NED origin => ECEF = %.6f  %.6f  %.6f (should be ~=0)\n', ...
        ned2ecef_vector([0 0 0], ref_lat, ref_lon));

% Ground truth in different frames
t_truth = truth(:,2);
pos_truth_ecef = truth(:,3:5);
vel_truth_ecef = truth(:,6:8);
acc_truth_ecef = [zeros(1,3); diff(vel_truth_ecef)./diff(t_truth)];

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

% Interpolate truth data to estimator time
% Then convert ECEF truth to the estimator NED frame for residuals
pos_truth_ecef_i = interp1(t_truth, pos_truth_ecef, t_est, 'linear', 'extrap');
vel_truth_ecef_i = interp1(t_truth, vel_truth_ecef, t_est, 'linear', 'extrap');
acc_truth_ecef_i = interp1(t_truth, acc_truth_ecef, t_est, 'linear', 'extrap');
pos_truth_ned_i_raw  = (C * (pos_truth_ecef_i' - ref_r0)).';
vel_truth_ned_i_raw  = (C*vel_truth_ecef_i.').';
acc_truth_ned_i_raw  = (C*acc_truth_ecef_i.').';

pos_truth_ned_i = centre(pos_truth_ned_i_raw .* sign_ned);
vel_truth_ned_i = vel_truth_ned_i_raw .* sign_ned;
acc_truth_ned_i = acc_truth_ned_i_raw .* sign_ned;

pos_gnss_ned_i_raw  = interp1(S.gnss_time, S.gnss_pos_ned,  t_est, 'linear', 'extrap');
vel_gnss_ned_i_raw  = interp1(S.gnss_time, S.gnss_vel_ned,  t_est, 'linear', 'extrap');
acc_gnss_ned_i_raw  = interp1(S.gnss_time, S.gnss_accel_ned, t_est, 'linear', 'extrap');

pos_gnss_ned_i = centre(pos_gnss_ned_i_raw .* sign_ned);
vel_gnss_ned_i = vel_gnss_ned_i_raw .* sign_ned;
acc_gnss_ned_i = acc_gnss_ned_i_raw .* sign_ned;

pos_gnss_ecef_i = interp1(S.gnss_time, S.gnss_pos_ecef,  t_est, 'linear', 'extrap');
vel_gnss_ecef_i = interp1(S.gnss_time, S.gnss_vel_ecef,  t_est, 'linear', 'extrap');
acc_gnss_ecef_i = interp1(S.gnss_time, S.gnss_accel_ecef, t_est, 'linear', 'extrap');

% Fused IMU results and derived acceleration
pos_ned_raw = S.pos_ned;
vel_ned_raw = S.vel_ned;
acc_ned_raw = [zeros(1,3); diff(vel_ned_raw)./diff(t_est)];

pos_ned = centre(pos_ned_raw .* sign_ned);
vel_ned = vel_ned_raw .* sign_ned;
acc_ned = [zeros(1,3); diff(vel_ned)./diff(t_est)];

% Downsample fused estimates to GNSS sample count
num_samples = size(S.x_log, 2);
n_gnss = size(gnss_pos_ned, 1);
downsample_factor = floor(num_samples / n_gnss);
time_idx = 1:downsample_factor:num_samples;
t_ds = t_est(time_idx);
pos_est_ds = S.x_log(1:3, time_idx);
vel_est_ds = S.x_log(4:6, time_idx);
pos_truth_ds = gnss_pos_ned';
vel_truth_ds = vel_truth_ned_i(time_idx, :)';
fprintf('Task 6: Downsampled estimates to %d samples (factor: %d)\n', numel(time_idx), downsample_factor);
if size(pos_truth_ds,2) ~= numel(time_idx)
    error('Task 6: Data length mismatch. Truth: %d, Estimated: %d. Adjust downsampling.', size(pos_truth_ds,2), numel(time_idx));
end
fprintf('Task 6: Validated data lengths: %d samples\n', numel(time_idx));

fprintf('Subtask 6.8.2: Plotted %s position North: First = %.4f, Last = %.4f\n', ...
    method, pos_est_ds(1,1), pos_est_ds(1,end));
fprintf('Subtask 6.8.2: Plotted %s position East: First = %.4f, Last = %.4f\n', ...
    method, pos_est_ds(2,1), pos_est_ds(2,end));
fprintf('Subtask 6.8.2: Plotted %s position Down: First = %.4f, Last = %.4f\n', ...
    method, pos_est_ds(3,1), pos_est_ds(3,end));
fprintf('Subtask 6.8.2: Plotted %s velocity North: First = %.4f, Last = %.4f\n', ...
    method, vel_est_ds(1,1), vel_est_ds(1,end));
fprintf('Subtask 6.8.2: Plotted %s velocity East: First = %.4f, Last = %.4f\n', ...
    method, vel_est_ds(2,1), vel_est_ds(2,end));
fprintf('Subtask 6.8.2: Plotted %s velocity Down: First = %.4f, Last = %.4f\n', ...
    method, vel_est_ds(3,1), vel_est_ds(3,end));

C_N_E = C';
pos_ecef = (C_N_E*pos_ned_raw')' + ref_r0';
vel_ecef = (C_N_E*vel_ned_raw')';
acc_ecef = (C_N_E*acc_ned_raw')';

% Body frame conversion
if ~exist('g_NED','var')
    g_NED = [0;0;constants.GRAVITY];
end
N = length(t_est);
pos_body = zeros(N,3); vel_body = zeros(N,3); acc_body = zeros(N,3);
pos_gnss_body = zeros(N,3); vel_gnss_body = zeros(N,3); acc_gnss_body = zeros(N,3);
pos_truth_body = zeros(N,3); vel_truth_body = zeros(N,3); acc_truth_body = zeros(N,3);
for k = 1:N
    C_B_N = euler_to_rot(S.euler_log(:,k));
    pos_body(k,:) = (C_B_N'*pos_ned_raw(k,:)')';
    vel_body(k,:) = (C_B_N'*vel_ned_raw(k,:)')';
    acc_body(k,:) = (C_B_N'*(acc_ned_raw(k,:)' - g_NED))';
    pos_gnss_body(k,:) = (C_B_N'*pos_gnss_ned_i_raw(k,:)')';
    vel_gnss_body(k,:) = (C_B_N'*vel_gnss_ned_i_raw(k,:)')';
    acc_gnss_body(k,:) = (C_B_N'*(acc_gnss_ned_i_raw(k,:)' - g_NED))';
    pos_truth_body(k,:) = (C_B_N'*pos_truth_ned_i_raw(k,:)')';
    vel_truth_body(k,:) = (C_B_N'*vel_truth_ned_i_raw(k,:)')';
    acc_truth_body(k,:) = (C_B_N'*(acc_truth_ned_i_raw(k,:)' - g_NED))';
end

plot_overlay('NED', run_id, t_est, pos_ned, vel_ned, acc_ned, ...
    t_est, pos_gnss_ned_i, vel_gnss_ned_i, acc_gnss_ned_i, ...
    t_est, pos_ned, vel_ned, acc_ned, out_dir, ...
    't_truth', t_est, 'pos_truth', pos_truth_ned_i, ...
    'vel_truth', vel_truth_ned_i, 'acc_truth', acc_truth_ned_i, ...
    'filename', sprintf('%s_task6_overlay_state_NED', run_id));

% Display downsampled NED overlay
fprintf('Task 6: Generating and displaying NED overlay plot...\n');
fig = figure('Name', 'Task 6 - NED State Overlay', 'Visible', 'on');
labels = {'North','East','Down'};
for idx = 1:3
    subplot(2,3,idx);
    plot(t_ds, pos_est_ds(idx,:), 'b', 'DisplayName', ['Est ' labels{idx}]);
    hold on;
    plot(t_ds, pos_truth_ds(idx,:), 'r--', 'DisplayName', ['Truth ' labels{idx}]);
    title(['Position ' labels{idx}]);
    if idx == 1
        ylabel('Position (m)');
    end
    grid on; legend('Location','best'); hold off;

    subplot(2,3,idx+3);
    plot(t_ds, vel_est_ds(idx,:), 'b', 'DisplayName', ['Est ' labels{idx}]);
    hold on;
    plot(t_ds, vel_truth_ds(idx,:), 'r--', 'DisplayName', ['Truth ' labels{idx}]);
    title(['Velocity ' labels{idx}]);
    xlabel('Time Step');
    if idx == 1
        ylabel('Velocity (m/s)');
    end
    grid on; legend('Location','best'); hold off;
end

output_file = fullfile(out_dir, sprintf('%s_task6_overlay_state_NED.pdf', run_id));
saveas(fig, output_file);
fprintf('Task 6: Saved overlay figure: %s\n', output_file);

plot_overlay('ECEF', run_id, t_est, pos_ecef, vel_ecef, acc_ecef, ...
    t_est, pos_gnss_ecef_i, vel_gnss_ecef_i, acc_gnss_ecef_i, ...
    t_est, pos_ecef, vel_ecef, acc_ecef, out_dir, ...
    't_truth', t_est, 'pos_truth', pos_truth_ecef_i, ...
    'vel_truth', vel_truth_ecef_i, 'acc_truth', acc_truth_ecef_i, ...
    'filename', sprintf('%s_task6_overlay_state_ECEF', run_id));

plot_overlay('Body', run_id, t_est, pos_body, vel_body, acc_body, ...
    t_est, pos_gnss_body, vel_gnss_body, acc_gnss_body, ...
    t_est, pos_body, vel_body, acc_body, out_dir, ...
    't_truth', t_est, 'pos_truth', pos_truth_body, ...
    'vel_truth', vel_truth_body, 'acc_truth', acc_truth_body, ...
    'filename', sprintf('%s_task6_overlay_state_Body', run_id));

files = dir(fullfile(out_dir, sprintf('%s_task6_overlay_state_*.pdf', run_id)));
fprintf('Task 6 overlay plots saved under: %s\n', out_dir);
for k = 1:numel(files)
    fprintf('  - %s\n', files(k).name);
end

% ------------------------------------------------------------------
% Compute overlay metrics for summary tables
% ------------------------------------------------------------------
[mNED, ~]  = compute_overlay_metrics(t_est, pos_ned,  vel_ned,  pos_truth_ned_i,  vel_truth_ned_i);
[mECEF, ~] = compute_overlay_metrics(t_est, pos_ecef, vel_ecef, pos_truth_ecef_i, vel_truth_ecef_i);
[mBody, ~] = compute_overlay_metrics(t_est, pos_body, vel_body, pos_truth_body,  vel_truth_body);
metrics = struct('NED', mNED, 'ECEF', mECEF, 'Body', mBody);
metrics_file = fullfile(out_dir, sprintf('%s_task6_metrics.mat', run_id));
save(metrics_file, 'metrics');
save(fullfile(out_dir, sprintf('%s_task6_results.mat', run_id)), ...
    'pos_est_ds', 'vel_est_ds', 'pos_truth_ds');
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
