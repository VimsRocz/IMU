function Task_6(task5_file, truth_file, tag)
%TASK_6  Overlay fused state (Task 5) with absolute truth.
%   TASK_6(TASK5_FILE, TRUTH_FILE, TAG) loads the fused navigation
%   solution produced by Task 5 and the matching STATE_X truth file.
%   Position and velocity components are compared in the NED, ECEF and
%   body frames.  Figures are saved under ``results/`` using ``TAG`` as
%   filename prefix:
%       results/<tag>_task6_overlay_state_<FRAME>.pdf
%       results/<tag>_task6_overlay_state_<FRAME>.png
%
%   The reference latitude/longitude are taken from
%   ``Task1_<dataset>_<method>.mat``.  ``TAG`` defaults to the prefix of
%   ``task5_file``.
%
%   Example:
%       Task_6('IMU_X001_GNSS_X001_TRIAD_task5_results.mat', ...
%             'STATE_X001.txt')
%
% This implementation mirrors the Python pipeline.

if nargin < 3 || isempty(tag)
    [~, tag] = fileparts(task5_file);
end

%% Load Task 5 fused data
S = load(task5_file, 'time', ...
    'pos_fused_ned','vel_fused_ned','acc_fused_ned', ...
    'pos_fused_ecef','vel_fused_ecef','acc_fused_ecef', ...
    'pos_fused_body','vel_fused_body','acc_fused_body');

if ~isfield(S,'time')
    error('time vector not found in %s', task5_file);
end

% Derive dataset and method from tag
m = regexp(tag, '(IMU_[^_]+_GNSS_[^_]+)_([A-Za-z0-9]+)', 'tokens', 'once');
if ~isempty(m)
    dataset_tag = m{1};
    method = m{2};
else
    dataset_tag = tag;
    method = 'METHOD';
end

%% Load reference origin from Task 1
task1_file = fullfile('results', sprintf('Task1_%s_%s.mat', dataset_tag, method));
lat0 = 0; lon0 = 0;
if isfile(task1_file)
    S1 = load(task1_file);
    if isfield(S1,'lat0_rad'); lat0 = S1.lat0_rad; end
    if isfield(S1,'lon0_rad'); lon0 = S1.lon0_rad; end
else
    warning('Task 6: Task 1 file %s not found. Using zeros for lat/lon.', task1_file);
end

%% Load absolute truth file
raw = readmatrix(truth_file);
truth_t = raw(:,2);
truth_pos_ecef = raw(:,3:5);
truth_vel_ecef = raw(:,6:8);
quat_e2b = raw(:,9:12);

[truth_pos_ned, truth_vel_ned] = ecef_to_ned_series(truth_pos_ecef, truth_vel_ecef, lat0, lon0);
C_e2b = quat_to_dcm(quat_e2b); % 3x3xN
truth_pos_body = multiprod(C_e2b, truth_pos_ecef); % Nx3
truth_vel_body = multiprod(C_e2b, truth_vel_ecef);

%% Time align truth to estimator time vector
est_t = S.time(:);
if abs(mean(diff(est_t)) - mean(diff(truth_t))) > 1e-3
    warning('Task 6: time step mismatch exceeds 1 ms');
end
pos_ned_i  = interp1(truth_t, truth_pos_ned, est_t, 'linear', 'extrap');
vel_ned_i  = interp1(truth_t, truth_vel_ned, est_t, 'linear', 'extrap');
pos_ecef_i = interp1(truth_t, truth_pos_ecef, est_t, 'linear', 'extrap');
vel_ecef_i = interp1(truth_t, truth_vel_ecef, est_t, 'linear', 'extrap');
pos_body_i = interp1(truth_t, truth_pos_body, est_t, 'linear', 'extrap');
vel_body_i = interp1(truth_t, truth_vel_body, est_t, 'linear', 'extrap');

%% Compute truth accelerations (aligned)
acc_ned_i  = gradient(vel_ned_i,  est_t);
acc_ecef_i = gradient(vel_ecef_i, est_t);
acc_body_i = gradient(vel_body_i, est_t);

% Convert to 3xN for plotting convenience
tt = est_t(:).';
pos_est_ned   = S.pos_fused_ned';   vel_est_ned   = S.vel_fused_ned';   acc_est_ned   = S.acc_fused_ned';
pos_truth_ned = pos_ned_i';         vel_truth_ned = vel_ned_i';         acc_truth_ned = acc_ned_i';
pos_est_ecef  = S.pos_fused_ecef';  vel_est_ecef  = S.vel_fused_ecef';  acc_est_ecef  = S.acc_fused_ecef';
pos_truth_ecef= pos_ecef_i';        vel_truth_ecef= vel_ecef_i';        acc_truth_ecef= acc_ecef_i';
pos_est_body  = S.pos_fused_body';  vel_est_body  = S.vel_fused_body';  acc_est_body  = S.acc_fused_body';
pos_truth_body= pos_body_i';        vel_truth_body= vel_body_i';        acc_truth_body= acc_body_i';

%% Error metrics (per-frame)
metrics.NED  = compute_overlay_metrics(est_t, S.pos_fused_ned,  S.vel_fused_ned,  pos_ned_i,  vel_ned_i);
metrics.ECEF = compute_overlay_metrics(est_t, S.pos_fused_ecef, S.vel_fused_ecef, pos_ecef_i, vel_ecef_i);
metrics.Body = compute_overlay_metrics(est_t, S.pos_fused_body, S.vel_fused_body, pos_body_i, vel_body_i);

fprintf('Frame       RMSEpos  FinalPos  RMSEvel  FinalVel  RMSEacc  FinalAcc\n');
fields = fieldnames(metrics);
for i = 1:numel(fields)
    m = metrics.(fields{i});
    fprintf('%-8s  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f\n', ...
        fields{i}, m.rmse_pos, m.final_pos, m.rmse_vel, m.final_vel, m.rmse_acc, m.final_acc);
end

%% Generate figures and save overlay data
outDir = 'results';
if ~exist(outDir,'dir'); mkdir(outDir); end

plot_state96('NED',  tt, pos_truth_ned,  vel_truth_ned,  acc_truth_ned,  pos_est_ned,  vel_est_ned,  acc_est_ned,  tag, outDir);
plot_state96('ECEF', tt, pos_truth_ecef, vel_truth_ecef, acc_truth_ecef, pos_est_ecef, vel_est_ecef, acc_est_ecef, tag, outDir);
plot_state96('Body', tt, pos_truth_body, vel_truth_body, acc_truth_body, pos_est_body, vel_est_body, acc_est_body, tag, outDir);

% Save snapshot for Task 7 reuse
save(fullfile(outDir,sprintf('%s_task6_overlay.mat',tag)), 'tt','pos_*','vel_*','acc_*');
save(fullfile(outDir,sprintf('%s_task6_metrics.mat',tag)), 'metrics');
end

%% --------------------------------------------------------------------
function plot_state96(frame, tt, pT, vT, aT, pF, vF, aF, tag, folder)
    % Create 3\x3 grid : (row1 = Position, row2 = Velocity, row3 = Accel)
    f  = figure('Color','w','Position',[50 50 1400 900]);
    tl = tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
    hdr = {'X','Y','Z'};      % will become \x0394N, \x0394E, \x0394D for NED automatically
    if strcmp(frame,'NED'), hdr = {'\x0394N [m]','\x0394E [m]','\x0394D [m]'}; end

    rows  = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
    cols  = {[0 0 0],[0 0.447 0.741]};   % black truth, blue fused

    for k = 1:3                      % ----- col loop (X/Y/Z)
        % ---- Position -----
        nexttile(3*(0)+k);  hold on;
        plot(tt,pT(k,:),'-','Color',cols{1},'LineWidth',1);
        plot(tt,pF(k,:),':','Color',cols{2},'LineWidth',1);
        title(hdr{k}); if k==1, ylabel(rows{1}); end; grid on;

        % ---- Velocity -----
        nexttile(3*(1)+k);  hold on;
        plot(tt,vT(k,:),'-','Color',cols{1},'LineWidth',1);
        plot(tt,vF(k,:),':','Color',cols{2},'LineWidth',1);
        if k==1, ylabel(rows{2}); end; grid on;

        % ---- Acceleration -----
        nexttile(3*(2)+k);  hold on;
        plot(tt,aT(k,:),'-','Color',cols{1},'LineWidth',1);
        plot(tt,aF(k,:),':','Color',cols{2},'LineWidth',1);
        if k==1, ylabel(rows{3}); end; grid on;
    end
    legend({'Truth','Fused GNSS+IMU'},'Location','best','Box','off');
    xlabel(tl,'Time [s]');
    sgtitle(sprintf('Task 6 \x2013 %s \x2013 %s Frame (Fused vs. Truth)', ...
                    strrep(tag,'_','\_'), frame));
    % save
    base = fullfile(folder,sprintf('%s_task6_overlay_state_%s',tag,frame));
    exportgraphics(f,[base '.png'],'Resolution',300);
    exportgraphics(f,[base '.pdf'],'ContentType','vector');
    close(f);
end

%% --------------------------------------------------------------------
function C = quat_to_dcm(q)
%QUAT_TO_DCM Convert quaternion [q0 q1 q2 q3] to DCM stack.
N = size(q,1);
C = zeros(3,3,N);
for k=1:N
    q0 = q(k,1); q1 = q(k,2); q2 = q(k,3); q3 = q(k,4);
    C(:,:,k) = [1-2*(q2^2+q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2); ...
                2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3 - q0*q1); ...
                2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1-2*(q1^2+q2^2)];
end
end

%% --------------------------------------------------------------------
function out = multiprod(C, V)
%MULTIPROD Multiply a stack of 3x3 matrices with 3D vectors.
N = size(C,3);
out = zeros(N,3);
for k=1:N
    out(k,:) = (C(:,:,k) * V(k,:).').';
end
end

%% --------------------------------------------------------------------
function [pos_ned, vel_ned] = ecef_to_ned_series(pos_ecef, vel_ecef, lat_rad, lon_rad)
C = compute_C_ECEF_to_NED(lat_rad, lon_rad);
r0 = pos_ecef(1,:);
pos_ned = (C * (pos_ecef - r0).').';
vel_ned = (C * vel_ecef.').';
end

