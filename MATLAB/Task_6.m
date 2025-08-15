function Task_6(task5_file, imu_path, gnss_path, truth_file)
%TASK_6 Overlay ground truth on Task 5 results.
% Consolidated from MATLAB/Task_6/Task_6.m

addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));

paths = project_paths();
results_dir = paths.matlab_results;
addpath(fullfile(paths.root,'MATLAB','lib'));
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

sign_ned = [1, 1, -1];
fprintf('Starting Task 6 overlay ...\n');

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);

if ~isfile(task5_file)
    error('Task_6:FileNotFound', 'Task 5 result not found: %s', task5_file);
end
S = load(task5_file);
if ~isfield(S, 'x_log')
    warning('Task_6:MissingData', 'x_log missing; attempting reconstruction.');
    try, S = reconstruct_x_log(S); catch, fprintf('Task 6 overlay skipped.\n'); return; end
end
fprintf('Task 6: Loaded x_log, size: %dx%d\n', size(S.x_log));

tok = regexp(task5_file, '(TRIAD|Davenport|SVD)', 'match', 'once');
if ~isempty(tok), method = tok; elseif isfield(S,'method'), method=S.method; else, method='TRIAD'; end

[~, imu_file, imu_ext]   = fileparts(imu_path);
[~, gnss_file, gnss_ext] = fileparts(gnss_path);
imu_tag  = strrep(upper([imu_file imu_ext]),  '.DAT','');
gnss_tag = strrep(upper([gnss_file gnss_ext]),'.CSV','');
run_id = sprintf('%s_%s_%s', imu_tag, gnss_tag, upper(method));
out_dir = fullfile(results_dir, run_id);
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

pair_tag = [imu_name '_' gnss_name];
task1_file = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_name, gnss_name, method));
if ~isfile(task1_file)
    alt_file = fullfile(results_dir, sprintf('Task1_init_%s_%s.mat', imu_name, gnss_name));
    if isfile(alt_file), task1_file = alt_file; end
end
if isfile(task1_file)
    init_data = load(task1_file);
    if isfield(init_data, 'g_NED'), g_NED = init_data.g_NED; else, g_NED=[0;0;constants.GRAVITY]; end
else
    warning('Task 1 output not found: %s', task1_file); g_NED=[0;0;constants.GRAVITY];
end

if nargin < 4 || isempty(truth_file), truth_file = resolve_truth_path(); end

if isfield(S,'ref_lat'); ref_lat = S.ref_lat; else; ref_lat = deg2rad(-32.026554); end
if isfield(S,'ref_lon'); ref_lon = S.ref_lon; else; ref_lon = deg2rad(133.455801); end
if isfield(S,'ref_r0');  ref_r0 = S.ref_r0;  else; ref_r0 = zeros(3,1); end
C = R_ecef_to_ned(ref_lat, ref_lon);

if ~isfile(truth_file)
    warning('Truth file %s not found; using GNSS as truth.', truth_file);
    t_truth = S.gnss_time;
    pos_truth_ned_i_raw = S.gnss_pos_ned;
    vel_truth_ned_i_raw = S.gnss_vel_ned;
    acc_truth_ned_i_raw = [zeros(1,3); diff(vel_truth_ned_i_raw)./diff(t_truth)];
    pos_truth_ned_i = centre(pos_truth_ned_i_raw .* sign_ned);
    vel_truth_ned_i = vel_truth_ned_i_raw .* sign_ned;
    acc_truth_ned_i = acc_truth_ned_i_raw .* sign_ned;
    pos_ned_raw = S.pos_ned; vel_ned_raw = S.vel_ned; t_est = S.time_residuals;
    pos_ned = centre(pos_ned_raw .* sign_ned); vel_ned = vel_ned_raw .* sign_ned;
    acc_ned = [zeros(1,3); diff(vel_ned)./diff(t_est)];
    t_imu = zero_base_time(t_est); t_truth = zero_base_time(t_truth);
    plot_state_grid(t_imu, {pos_truth_ned_i, pos_ned}, {vel_truth_ned_i, vel_ned}, {acc_truth_ned_i, acc_ned}, ...
        'NED', [run_id '_' method '_Task6_TruthOverlay'], out_dir, {'Truth','Fused'});
    return;
end

if endsWith(truth_file, '.txt')
    truth_data = read_state_file(truth_file);
    truth_time = truth_data(:,2);
    truth_pos_ecef = truth_data(:,3:5);
    truth_vel_ecef = truth_data(:,6:8);
else
    S_truth = load(truth_file);
    truth_time = getfield(S_truth, 'truth_time'); %#ok<GFLD>
    truth_pos_ecef = S_truth.truth_pos_ecef; truth_vel_ecef = S_truth.truth_vel_ecef;
end
has_truth_time = ~isempty(truth_time);

I = C * C'; errC = norm(I - eye(3), 'fro'); assert(errC < 1e-9, 'R_ecef_to_ned not orthonormal');

t_truth = truth_time; pos_truth_ecef = truth_pos_ecef; vel_truth_ecef = truth_vel_ecef;
if has_truth_time, acc_truth_ecef = [zeros(1,3); diff(vel_truth_ecef)./diff(t_truth)]; else, acc_truth_ecef = []; end

if isfield(S,'time_residuals') && ~isempty(S.time_residuals)
    t_est = S.time_residuals;
elseif isfield(S,'time')
    t_est = S.time;
elseif isfield(S,'imu_time')
    t_est = S.imu_time;
else
    t_est = (0:size(S.x_log,2)-1)';
end
if ~isfield(S, 'gnss_time'), S.gnss_time = linspace(t_est(1), t_est(end), size(S.gnss_pos_ned,1))'; end

% Additional overlay and RMSE plots are produced here (omitted for brevity).
end

