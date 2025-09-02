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

addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));

paths = project_paths();
results_dir = paths.matlab_results;
addpath(fullfile(paths.root,'MATLAB','lib'));
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
cfg = default_cfg();
% Always pop Task 6 figures (interactive)
visibleFlag = 'on';

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

% Echo estimator attitude convention for clarity in logs
if isfield(S,'att_quat')
    fprintf('Estimator attitude convention: C_nb (Body->NED), quaternion [w x y z] from att_quat (Nx4)\n');
else
    warning('Task6: att_quat missing in Task 5 results; attitude overlays will be skipped if required.');
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
% Compute run_id locally to avoid dependency on path setup
[~, imu_file, imu_ext]   = fileparts(imu_path);
[~, gnss_file, gnss_ext] = fileparts(gnss_path);
imu_tag  = strrep(upper([imu_file imu_ext]),  '.DAT','');
gnss_tag = strrep(upper([gnss_file gnss_ext]),'.CSV','');
rid = sprintf('%s_%s_%s', imu_tag, gnss_tag, upper(method));
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
dt_shift = 0;
if isfield(S,'dt_truth_shift'); dt_shift = S.dt_truth_shift; end % FIX: use Task5 time shift
C = R_ecef_to_ned(ref_lat, ref_lon);
C_N_E = C';

if ~isfile(truth_file)
    warning('Truth file %s not found; using GNSS as truth.', truth_file);
    t_truth = S.gnss_time + dt_shift; % FIX: apply time shift
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
    % Read full truth state file (count, time, X Y Z, VX VY VZ, q)
    truth_data = read_state_file(truth_file);
    truth_time = truth_data(:,2) + dt_shift; % FIX: apply time shift
    truth_pos_ecef = truth_data(:,3:5);
    truth_vel_ecef = truth_data(:,6:8);
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
errC = norm(I - eye(3), 'fro');
assert(errC < 1e-9, 'R_ecef_to_ned not orthonormal (||C*C''-I||=%.3e)', errC);

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

% Require and load estimator attitude quaternion from Task 5
if ~isfield(S,'att_quat') || size(S.att_quat,2) ~= 4
    error('Task6: att_quat (Nx4, wxyz) missing from Task 5 results; cannot plot attitude overlays.');
end
q_est = S.att_quat; % [N x 4], wxyz, Body->NED
fprintf('[Task6] Using estimator attitude from att_quat (Nx4, wxyz). Size=%dx4\n', size(q_est,1));
if size(q_est,1) >= 2
    fprintf('[Task6] att_quat first=[% .4f % .4f % .4f % .4f], last=[% .4f % .4f % .4f % .4f]\n', ...
        q_est(1,1), q_est(1,2), q_est(1,3), q_est(1,4), q_est(end,1), q_est(end,2), q_est(end,3), q_est(end,4));
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

% Debug: prove Task-6 uses att_quat by plotting last 60s of q_est components
try
    t_rel = t_est - t_est(1);
    win = 60; mask = t_rel >= max(0, t_rel(end) - win);
    if any(mask)
        fdbg = figure('Visible','off','Position',[100 100 700 300]);
        plot(t_rel(mask), q_est(mask,1), 'LineWidth',1.1); hold on;
        plot(t_rel(mask), q_est(mask,2:4), 'LineWidth',1.1);
        grid on; xlabel('Time [s]'); ylabel('q components');
        title('Task-6 q\_est (from att\_quat), Body\rightarrowNED');
        legend('q0','q1','q2','q3','Location','best');
        print(fdbg, fullfile(out_dir, sprintf('%s_task6_q_est_from_att_quat_last60s_NED.png', run_id)), '-dpng','-r200');
        close(fdbg);
        fprintf('Task 6: saved q\_est debug plot (last 60s).\n');
    end
catch ME
    warning('Task 6: failed to save q\_est debug plot: %s', ME.message);
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

% 3x3 overlay grid comparing fused vs truth
fused_struct = struct('t', t_est, 'pos', pos_ned, 'vel', vel_ned, 'acc', acc_ned);
% For interactive overlays, plot the full truth timeline
truth_struct = struct('t', t_truth, 'pos', pos_truth_ned_i_raw, ...
                      'vel', vel_truth_ned_i_raw, 'acc', acc_truth_ned_i_raw);
% Use estimator time as the reference; truth has already been
% interpolated/aligned to t_est above when needed.
t_ref = t_imu;
truth_struct.t = t_est;
hfig_ned = plot_state_grid_overlay(t_ref, fused_struct, truth_struct, 'NED', ...
    'Title', sprintf('%s Task6: Fused vs Truth', run_id), ...
    'Visible', visibleFlag, 'MaxPlotPoints', 20000);
set(hfig_ned,'Units','centimeters','Position',[2 2 18 9]); % FIX: page width export
set(hfig_ned,'PaperPositionMode','auto');
exportgraphics(hfig_ned, fullfile(out_dir, sprintf('%s_task6_overlay_grid_NED.png', run_id)), 'Resolution',300);

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

% Body frame conversion skipped (truth attitude not used)
pos_body = [];
vel_body = [];
acc_body = [];
pos_truth_body = [];
vel_truth_body = [];
acc_truth_body = [];

% Persist processed TRUTH and FUSED series in all three frames for Task 6
try
    truth_proc_file = fullfile(out_dir, sprintf('%s_task6_truth_processed.mat', run_id));
    fused_proc_file = fullfile(out_dir, sprintf('%s_task6_fused_processed.mat', run_id));
    % Backwards/compat names expected by user requests (body skipped)
    % TRUTH (ECEF/NED)
    save(truth_proc_file, 't_est', ...
        'pos_truth_ecef_i', 'vel_truth_ecef_i', 'acc_truth_ecef_i', ...
        'pos_truth_ecef',   'vel_truth_ecef',   'acc_truth_ecef', ...
        'pos_truth_ned_i_raw', 'vel_truth_ned_i_raw', 'acc_truth_ned_i_raw', ...
        'pos_truth_ned_i',     'vel_truth_ned_i',     'acc_truth_ned_i', ...
        'ref_lat', 'ref_lon', 'ref_r0');
    % FUSED (ECEF/NED)
    save(fused_proc_file, 't_est', ...
        'pos_ned', 'vel_ned', 'acc_ned', ...
        'pos_ecef', 'vel_ecef', 'acc_ecef', ...
        'ref_lat', 'ref_lon', 'ref_r0');
    fprintf('Task 6: saved processed TRUTH (%s) and FUSED (%s)\n', truth_proc_file, fused_proc_file);
catch ME
    warning('Task 6: failed to save processed TRUTH/FUSED: %s', ME.message);
end

% Save consolidated Task 6 results at standard path for downstream tasks
try
    results6 = struct();
    results6.t_est = t_est;
    results6.t_truth_full = t_truth;
    % FUSED
    results6.pos_ned = pos_ned; results6.vel_ned = vel_ned; results6.acc_ned = acc_ned;
    results6.pos_ecef = pos_ecef; results6.vel_ecef = vel_ecef; results6.acc_ecef = acc_ecef;
    % TRUTH full
    results6.truth_pos_ned = pos_truth_ned_i_raw; results6.truth_vel_ned = vel_truth_ned_i_raw; results6.truth_acc_ned = acc_truth_ned_i_raw;
    results6.truth_pos_ecef = pos_truth_ecef; results6.truth_vel_ecef = vel_truth_ecef; results6.truth_acc_ecef = acc_truth_ecef;
    % Refs
    results6.ref_lat = ref_lat; results6.ref_lon = ref_lon; results6.ref_r0 = ref_r0; results6.dt_truth_shift = dt_shift;
    results6.method = method;
    res6_sub = fullfile(out_dir, sprintf('%s_task6_results.mat', run_id));
    res6_root = fullfile(results_dir, sprintf('%s_task6_results.mat', run_id));
    save(res6_sub, '-struct', 'results6');
    save(res6_root, '-struct', 'results6');
    % Expose in base workspace for interactive exploration
    assignin('base', 'task6_results', results6);
    % Also save via standard helper for consistency with other tasks
    try
        save_task_results(results6, imu_name, gnss_name, method, 6);
    catch
    end
    fprintf('Task 6: saved results to %s and %s\n', res6_sub, res6_root);
catch ME
    warning('Task 6: failed to save task6_results: %s', ME.message);
end

save_overlay_state(t_est, pos_ned, vel_ned, pos_truth_ned_i, vel_truth_ned_i, ...
    'NED', run_id, method, out_dir);
save_overlay_state(t_est, pos_ecef, vel_ecef, pos_truth_ecef_i, vel_truth_ecef_i, ...
    'ECEF', run_id, method, out_dir);

% Also plot and save ECEF and Body frame overlays as 3x3 grids (popup if requested)
% ECEF overlay
truth_ecef_struct = struct('t', t_truth, 'pos', pos_truth_ecef, 'vel', vel_truth_ecef, 'acc', acc_truth_ecef);
fused_ecef_struct  = struct('t', t_est, 'pos', pos_ecef,          'vel', vel_ecef,          'acc', acc_ecef);
hfig_ecef = plot_state_grid_overlay(t_est, fused_ecef_struct, truth_ecef_struct, 'ECEF', ...
    'Title', sprintf('%s Task6: Fused vs Truth', run_id), ...
    'Visible', visibleFlag, 'MaxPlotPoints', 20000);
set(hfig_ecef,'Units','centimeters','Position',[2 2 18 9]); % FIX: page width export
set(hfig_ecef,'PaperPositionMode','auto');
exportgraphics(hfig_ecef, fullfile(out_dir, sprintf('%s_task6_overlay_grid_ECEF.png', run_id)), 'Resolution',300);

% ------------------------------------------------------------------
% Compute overlay metrics for summary tables
% ------------------------------------------------------------------
[mNED, ~]  = compute_overlay_metrics(t_est, pos_ned,  vel_ned,  pos_truth_ned_i,  vel_truth_ned_i);
[mECEF, ~] = compute_overlay_metrics(t_est, pos_ecef, vel_ecef, pos_truth_ecef_i, vel_truth_ecef_i);
metrics = struct('NED', mNED, 'ECEF', mECEF);
metrics_file = fullfile(out_dir, sprintf('%s_task6_metrics.mat', run_id));
try
    save(metrics_file, 'metrics');
catch ME
    warning('Failed to save metrics to %s: %s', metrics_file, ME.message);
end
rows = {
    'NED',  mNED.rmse_pos,  mNED.final_pos,  mNED.rmse_vel,  mNED.final_vel,  mNED.rmse_acc,  mNED.final_acc;
    'ECEF', mECEF.rmse_pos, mECEF.final_pos, mECEF.rmse_vel, mECEF.final_vel, mECEF.rmse_acc, mECEF.final_acc};
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

%% ========================================================================
% Additional comparisons: GNSS vs IMU (raw + integrated) in NED/ECEF/Body
% =========================================================================
try
    % NED comparison
    t_g = S.gnss_time;
    pos_g = S.gnss_pos_ned; vel_g = S.gnss_vel_ned;
    acc_g = [zeros(1,3); diff(vel_g)./diff(t_g)];
    fprintf('Task 6: NED comparison plotting | GNSS samples=%d | est samples=%d\n', numel(t_g), numel(t_est));
    fig = figure('Name','Task6 NED Comparison','Position',[100 100 1200 900], 'Visible', visibleFlag);
    labels = {'North','East','Down'};
    for k = 1:3
        subplot(3,3,k); hold on; plot(t_g, pos_g(:,k),'k:','DisplayName','GNSS'); plot(t_est, pos_ned(:,k),'b-','DisplayName','IMU fused'); grid on; title(['Pos ' labels{k}]); legend;
        subplot(3,3,3+k); hold on; plot(t_g, vel_g(:,k),'k:'); plot(t_est, vel_ned(:,k),'b-'); grid on; title(['Vel ' labels{k}]);
        subplot(3,3,6+k); hold on; plot(t_g, acc_g(:,k),'k:'); plot(t_est, acc_ned(:,k),'b-'); grid on; title(['Acc ' labels{k}]);
    end
    set(fig,'PaperPositionMode','auto');
    ned_base = fullfile(out_dir, sprintf('%s_task6_compare_NED', run_id));
    print(fig, [ned_base '.pdf'], '-dpdf', '-bestfit');
    print(fig, [ned_base '.png'], '-dpng');
    fprintf('Task 6: saved NED comparison plots to %s.[pdf|png]\n', ned_base);
    close(fig);

    % ECEF comparison
    fprintf('Task 6: ECEF comparison plotting | samples=%d\n', numel(t_est));
    fig = figure('Name','Task6 ECEF Comparison','Position',[100 100 1200 900], 'Visible', visibleFlag);
    labelsE = {'X','Y','Z'};
    pos_f = pos_ecef; vel_f = vel_ecef; acc_f = acc_ecef; % from earlier
    if isfield(S,'gnss_pos_ecef'), pos_ge = S.gnss_pos_ecef; else, pos_ge = (C_N_E*pos_g')'+ref_r0'; end
    if isfield(S,'gnss_vel_ecef'), vel_ge = S.gnss_vel_ecef; else, vel_ge = (C_N_E*vel_g')'; end
    if isfield(S,'gnss_accel_ecef'), acc_ge = S.gnss_accel_ecef; else, acc_ge = [zeros(1,3); diff(vel_ge)./diff(t_g)]; end
    for k = 1:3
        subplot(3,3,k); hold on; plot(t_g, pos_ge(:,k),'k:','DisplayName','GNSS'); plot(t_est, pos_f(:,k),'b-','DisplayName','IMU fused'); grid on; title(['Pos ' labelsE{k}]); legend;
        subplot(3,3,3+k); hold on; plot(t_g, vel_ge(:,k),'k:'); plot(t_est, vel_f(:,k),'b-'); grid on; title(['Vel ' labelsE{k}]);
        subplot(3,3,6+k); hold on; plot(t_g, acc_ge(:,k),'k:'); plot(t_est, acc_f(:,k),'b-'); grid on; title(['Acc ' labelsE{k}]);
    end
    set(fig,'PaperPositionMode','auto');
    ecef_base = fullfile(out_dir, sprintf('%s_task6_compare_ECEF', run_id));
    print(fig, [ecef_base '.pdf'], '-dpdf', '-bestfit');
    print(fig, [ecef_base '.png'], '-dpng');
    fprintf('Task 6: saved ECEF comparison plots to %s.[pdf|png]\n', ecef_base);
    close(fig);

    % Body comparison (includes raw IMU acceleration if available)
    fprintf('Task 6: Body comparison plotting | samples=%d\n', numel(t_est));
    fig = figure('Name','Task6 Body Comparison','Position',[100 100 1200 900], 'Visible', visibleFlag);
    labelsB = {'X','Y','Z'};
    % Body comparison skipped
catch ME
    warning('Task 6 comparison plots failed: %s', ME.message);
end
end

function y = centre(x)
%CENTRE Remove the mean from each column vector.
    y = x - mean(x,1);
end
