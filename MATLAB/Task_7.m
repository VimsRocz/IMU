function Task_7(task5_matfile, truth_file, run_tag)
%TASK_7 Residual analysis and summary using fused and truth data.
%   TASK_7(TASK5_MATFILE, TRUTH_FILE, OUTPUT_TAG) loads the fused
%   navigation solution saved by Task 5 together with the reference
%   trajectory and computes residual position and velocity in the NED,
%   ECEF and Body frames. Diagnostic plots are displayed and stored under
%   ``results/`` with Python-style filenames. A summary structure with
%   RMSE statistics is also written to ``results/``.
%
%   Example:
%       Task_7('results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat', ...
%              'STATE_X001.txt', 'IMU_X002_GNSS_X002_TRIAD');

fprintf('Task 7: Residuals & Summary Analysis...\n');

if ~exist('results','dir'); mkdir('results'); end

%% --------------------------------------------------------------------
% Load fused estimator results
%% --------------------------------------------------------------------
if ~isfile(task5_matfile)
    error('Task 7: Fused result file not found: %s', task5_matfile);
end
S = load(task5_matfile);
if isfield(S,'pos_est_ned'); pos_est_ned = S.pos_est_ned; elseif isfield(S,'pos_ned'); pos_est_ned = S.pos_ned; else; error('Task 7: pos_est_ned missing'); end
if isfield(S,'vel_est_ned'); vel_est_ned = S.vel_est_ned; elseif isfield(S,'vel_ned'); vel_est_ned = S.vel_ned; else; error('Task 7: vel_est_ned missing'); end
if isfield(S,'pos_est_ecef') && isfield(S,'vel_est_ecef')
    pos_est_ecef = S.pos_est_ecef; vel_est_ecef = S.vel_est_ecef;
else
    if isfield(S,'lat0_rad'); lat = S.lat0_rad; else; lat = S.ref_lat_rad; end
    if isfield(S,'lon0_rad'); lon = S.lon0_rad; else; lon = S.ref_lon_rad; end
    if isfield(S,'r0'); r0 = S.r0; else; r0 = S.ref_r0; end
    C = compute_C_NED_to_ECEF(lat, lon);
    pos_est_ecef = (C * pos_est_ned')' + r0(:)';
    vel_est_ecef = (C * vel_est_ned')';
end
if isfield(S,'C_b_n'); C_b_n = S.C_b_n; else; C_b_n = repmat(eye(3),1,1,size(pos_est_ned,1)); end
if isfield(S,'time'); t_est = S.time(:); elseif isfield(S,'time_s'); t_est = S.time_s(:); else; t_est = (0:size(pos_est_ned,1)-1)'; end
if isfield(S,'time'); t_est = S.time(:); elseif isfield(S,'time_s'); t_est = S.time_s(:); else; t_est = (0:size(pos_est_ned,1)-1)'; end
fprintf('Task 7: Loaded estimator data from %s (%d samples)\n', task5_matfile, numel(t_est));

%% --------------------------------------------------------------------
% Load truth trajectory
%% --------------------------------------------------------------------
if ~isfile(truth_file)
    error('Task 7: Truth file not found: %s', truth_file);
end
try
    if endsWith(truth_file,'.txt')
        raw = read_state_file(truth_file);
        t_truth = raw(:,2);
        pos_truth_ecef = raw(:,3:5);
        vel_truth_ecef = raw(:,6:8);
    else
        T = load(truth_file);
        t_truth = T.time(:);
        pos_truth_ecef = T.pos_truth_ecef;
        vel_truth_ecef = T.vel_truth_ecef;
    end
catch ME
    error('Task 7: Failed to load truth data from %s (%s)', truth_file, ME.message);
end
fprintf('Task 7: Loaded truth data from %s (%d samples)\n', truth_file, numel(t_truth));

%% --------------------------------------------------------------------
% Derive reference frame and convert truth to NED
%% --------------------------------------------------------------------
ref_ecef = pos_truth_ecef(1,:); % origin
[lat_deg, lon_deg, ~] = ecef_to_geodetic(ref_ecef(1), ref_ecef(2), ref_ecef(3));
C_e_n = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
pos_truth_ned = (C_e_n * (pos_truth_ecef' - ref_ecef'))';
vel_truth_ned = (C_e_n * vel_truth_ecef')';

%% Interpolate truth to estimator timeline when lengths differ
if numel(t_truth) ~= numel(t_est)
    pos_truth_ned  = interp1(t_truth, pos_truth_ned,  t_est, 'linear', 'extrap');
    vel_truth_ned  = interp1(t_truth, vel_truth_ned,  t_est, 'linear', 'extrap');
    pos_truth_ecef = interp1(t_truth, pos_truth_ecef, t_est, 'linear', 'extrap');
    vel_truth_ecef = interp1(t_truth, vel_truth_ecef, t_est, 'linear', 'extrap');
    t = t_est;
    fprintf('Task 7: Interpolated truth data to estimator timeline\n');
else
    t = t_truth;
end

%% --------------------------------------------------------------------
% Convert estimates and truth to body frame
%% --------------------------------------------------------------------
if ndims(C_b_n) == 3
    n = size(pos_est_ned,1);
    pos_est_body  = zeros(n,3); pos_truth_body  = zeros(n,3);
    vel_est_body  = zeros(n,3); vel_truth_body  = zeros(n,3);
    for i = 1:n
        R = C_b_n(:,:,min(i,end));
        pos_est_body(i,:)  = (R' * pos_est_ned(i,:)')';
        pos_truth_body(i,:) = (R' * pos_truth_ned(i,:)')';
        vel_est_body(i,:)  = (R' * vel_est_ned(i,:)')';
        vel_truth_body(i,:) = (R' * vel_truth_ned(i,:)')';
    end
else
    R = C_b_n;
    pos_est_body  = (R' * pos_est_ned')';
    pos_truth_body = (R' * pos_truth_ned')';
    vel_est_body  = (R' * vel_est_ned')';
    vel_truth_body = (R' * vel_truth_ned')';
end
fprintf('Task 7: Converted all positions/velocities to body frame.\n');

%% --------------------------------------------------------------------
% Residuals in each frame
%% --------------------------------------------------------------------
res.ned.pos  = pos_est_ned  - pos_truth_ned;
res.ned.vel  = vel_est_ned  - vel_truth_ned;
res.ecef.pos = pos_est_ecef - pos_truth_ecef;
res.ecef.vel = vel_est_ecef - vel_truth_ecef;
res.body.pos = pos_est_body - pos_truth_body;
res.body.vel = vel_est_body - vel_truth_body;
fprintf('Task 7: Computed residuals for all frames.\n');

%% --------------------------------------------------------------------
% Plotting helpers
%% --------------------------------------------------------------------
frames = fieldnames(res);
labels_ned  = {'North','East','Down'};
labels_ecef = {'X','Y','Z'};
for fIdx = 1:numel(frames)
    frame = frames{fIdx};
    switch frame
        case 'ned';  labels = labels_ned;
        case 'ecef'; labels = labels_ecef;
        otherwise;   labels = {'X','Y','Z'};
    end
    plot_residuals(t, res.(frame).pos, res.(frame).vel, labels, frame, run_tag);
    plot_error_norms(t, res.(frame).pos, res.(frame).vel, frame, run_tag);
    plot_diff(t, res.(frame).pos, res.(frame).vel, labels, frame, run_tag);
    compute_summary(res.(frame).pos, res.(frame).vel, frame, run_tag);
end

fprintf('Task 7 complete: Residuals, error norms, and summary metrics generated and saved for all frames.\n');
end

%% =====================================================================
function plot_residuals(t, pos_r, vel_r, labels, frame, tag)
    fig = figure('Name', ['Task 7 Residuals ' upper(frame)], 'Visible','on');
    for k = 1:3
        subplot(2,3,k); plot(t, pos_r(:,k)); grid on;
        title(['Pos ' labels{k}]); ylabel('Residual [m]');
        if k==1; xlabel('Time [s]'); end
        subplot(2,3,k+3); plot(t, vel_r(:,k)); grid on;
        title(['Vel ' labels{k}]); ylabel('Residual [m/s]'); xlabel('Time [s]');
    end
    sgtitle(['Residuals (' upper(frame) ' frame)']);
    pdf = fullfile('results', sprintf('%s_task7_3_residuals_position_velocity_%s.pdf', tag, frame));
    png = fullfile('results', sprintf('%s_task7_3_residuals_position_velocity_%s.png', tag, frame));
    saveas(fig, pdf); saveas(fig, png);
    fprintf('Saved residuals plot for %s frame.\n', frame);
end

function plot_error_norms(t, pos_r, vel_r, frame, tag)
    fig = figure('Name', ['Task 7 Error Norms ' upper(frame)], 'Visible','on');
    plot(t, vecnorm(pos_r,2,2), 'DisplayName','|pos|'); hold on;
    plot(t, vecnorm(vel_r,2,2), 'DisplayName','|vel|');
    grid on; legend; xlabel('Time [s]'); ylabel('Error Norm');
    title(['Error Norms (' upper(frame) ' frame)']);
    pdf = fullfile('results', sprintf('%s_task7_3_error_norms_%s.pdf', tag, frame));
    png = fullfile('results', sprintf('%s_task7_3_error_norms_%s.png', tag, frame));
    saveas(fig, pdf); saveas(fig, png);
    fprintf('Saved error norm plot for %s frame.\n', frame);
end

function plot_diff(t, pos_r, vel_r, labels, frame, tag)
    fig = figure('Name', ['Task 7 Differences ' upper(frame)], 'Visible','on');
    thr = 1.0;
    for k = 1:3
        subplot(2,3,k); plot(t, pos_r(:,k)); hold on; yline(thr,'r--'); yline(-thr,'r--'); grid on;
        title(['Pos ' labels{k}]); ylabel('Diff [m]');
        subplot(2,3,k+3); plot(t, vel_r(:,k)); hold on; yline(thr,'r--'); yline(-thr,'r--'); grid on;
        title(['Vel ' labels{k}]); ylabel('Diff [m/s]'); xlabel('Time [s]');
    end
    sgtitle(['Truth - Fused Difference (' upper(frame) ' frame)']);
    pdf = fullfile('results', sprintf('%s_task7_5_diff_truth_fused_over_time_%s.pdf', tag, frame));
    png = fullfile('results', sprintf('%s_task7_5_diff_truth_fused_over_time_%s.png', tag, frame));
    saveas(fig, pdf); saveas(fig, png);
    fprintf('Saved difference over time plot for %s frame.\n', frame);
end

function compute_summary(pos_r, vel_r, frame, tag)
    rmse_pos = sqrt(mean(sum(pos_r.^2,2))); %# overall RMSE
    final_pos = norm(pos_r(end,:));
    rmse_vel = sqrt(mean(sum(vel_r.^2,2)));
    final_vel = norm(vel_r(end,:));
    rms_resid_pos = sqrt(mean(pos_r.^2,'all'));
    rms_resid_vel = sqrt(mean(vel_r.^2,'all'));
    max_resid_pos = max(vecnorm(pos_r,2,2));
    max_resid_vel = max(vecnorm(vel_r,2,2));
    summary = struct('rmse_pos', rmse_pos, 'final_pos', final_pos, ...
        'rmse_vel', rmse_vel, 'final_vel', final_vel, ...
        'rms_resid_pos', rms_resid_pos, 'rms_resid_vel', rms_resid_vel, ...
        'max_resid_pos', max_resid_pos, 'max_resid_vel', max_resid_vel);
    txt = fullfile('results', sprintf('%s_task7_summary_%s.txt', tag, frame));
    fid = fopen(txt,'w');
    fprintf(fid,'RMSE position [m]: %.4f\n', rmse_pos);
    fprintf(fid,'RMSE velocity [m/s]: %.4f\n', rmse_vel);
    fprintf(fid,'Final position error [m]: %.4f\n', final_pos);
    fprintf(fid,'Final velocity error [m/s]: %.4f\n', final_vel);
    fprintf(fid,'RMS resid pos [m]: %.4f max resid pos [m]: %.4f\n', rms_resid_pos, max_resid_pos);
    fprintf(fid,'RMS resid vel [m/s]: %.4f max resid vel [m/s]: %.4f\n', rms_resid_vel, max_resid_vel);
    fclose(fid);
    mat = fullfile('results', sprintf('%s_task7_summary_%s.mat', tag, frame));
    save(mat, 'summary');
    fprintf('Saved summary metrics for %s frame.\n', frame);
    if strcmpi(frame,'ned')
        runtime = t(end)-t(1);
        parts = strsplit(tag,'_');
        method = parts{end};
        imu_file = [parts{1} '.dat'];
        gnss_file = [parts{2} '.csv'];
        summary_line = sprintf(['[SUMMARY] method=%s imu=%s gnss=%s rmse_pos=%.2fm final_pos=%.2fm ' ...
            'rms_resid_pos=%.2fm max_resid_pos=%.2fm rms_resid_vel=%.2fm max_resid_vel=%.2fm runtime=%.2fs'], ...
            method, imu_file, gnss_file, rmse_pos, final_pos, rms_resid_pos, max_resid_pos, rms_resid_vel, max_resid_vel, runtime);
        fprintf('%s\n', summary_line);
    end
end
