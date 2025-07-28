function Task_6(task5_file, truth_file, tag)
%TASK_6 Overlay fused estimator output with ground truth.
%   TASK_6(TASK5_FILE, TRUTH_FILE, TAG) loads the results saved by
%   ``Task_5`` and the STATE_X truth file then plots fused versus truth
%   trajectories in the NED, ECEF and body frames. Three figures are
%   saved under the repository ``results`` directory using ``TAG`` as the
%   filename prefix. A ``*_task6_stats.mat`` file containing RMSE and
%   final error metrics is also written.
%
%   Usage:
%       Task_6('Task5_IMU_X001_GNSS_X001_TRIAD.mat','STATE_X001.txt','X001_TRIAD')
%
%   This function mirrors the Python implementation and keeps the MATLAB
%   pipeline in sync.

if nargin < 3 || isempty(tag)
    [~,tag] = fileparts(task5_file);
end
results_dir = get_results_dir();
if ~exist(results_dir,'dir'); mkdir(results_dir); end

S = load(task5_file);
T = read_truth_file(truth_file);

% Reference coordinates
if isfield(S,'ref_lat_rad'); lat0 = S.ref_lat_rad; elseif isfield(S,'ref_lat'); lat0 = S.ref_lat; else; lat0 = 0; end
if isfield(S,'ref_lon_rad'); lon0 = S.ref_lon_rad; elseif isfield(S,'ref_lon'); lon0 = S.ref_lon; else; lon0 = 0; end
if isfield(S,'ref_r0_ecef'); r0 = S.ref_r0_ecef(:); elseif isfield(S,'ref_r0'); r0 = S.ref_r0(:); else; r0 = [0;0;0]; end

time = S.time(:);

pos_truth_ned = [T.pos_N, T.pos_E, T.pos_D];
vel_truth_ned = [T.vel_N, T.vel_E, T.vel_D];
[pos_truth_ecef, vel_truth_ecef] = ned2ecef_series(pos_truth_ned, vel_truth_ned, lat0, lon0, r0);

n = numel(time);
pos_truth_body = zeros(n,3); vel_truth_body = zeros(n,3);
for k = 1:n
    C_BN = S.C_BN_log(:,:,k); % body<-NED
    pos_truth_body(k,:) = (C_BN*pos_truth_ned(k,:)').';
    vel_truth_body(k,:) = (C_BN*vel_truth_ned(k,:)').';
end

% Plot overlays
plot_overlay(time, pos_truth_ned, vel_truth_ned, S.pos_fused_ned, S.vel_fused_ned,
    'Task 6 – Fused vs. Truth (NED)', fullfile(results_dir,[tag '_task6_overlay_state_NED']));
plot_overlay(time, pos_truth_ecef, vel_truth_ecef, S.pos_fused_ecef, S.vel_fused_ecef,
    'Task 6 – Fused vs. Truth (ECEF)', fullfile(results_dir,[tag '_task6_overlay_state_ECEF']));
plot_overlay(time, pos_truth_body, vel_truth_body, S.pos_fused_body, S.vel_fused_body,
    'Task 6 – Fused vs. Truth (Body)', fullfile(results_dir,[tag '_task6_overlay_state_Body']));

% Error metrics
rmse_pos_ned  = sqrt(mean(sum((S.pos_fused_ned - pos_truth_ned).^2,2)));
rmse_vel_ned  = sqrt(mean(sum((S.vel_fused_ned - vel_truth_ned).^2,2)));
rmse_pos_ecef = sqrt(mean(sum((S.pos_fused_ecef - pos_truth_ecef).^2,2)));
rmse_vel_ecef = sqrt(mean(sum((S.vel_fused_ecef - vel_truth_ecef).^2,2)));
rmse_pos_body = sqrt(mean(sum((S.pos_fused_body - pos_truth_body).^2,2)));
rmse_vel_body = sqrt(mean(sum((S.vel_fused_body - vel_truth_body).^2,2)));
final_pos_ned  = norm(S.pos_fused_ned(end,:) - pos_truth_ned(end,:));
final_vel_ned  = norm(S.vel_fused_ned(end,:) - vel_truth_ned(end,:));
final_pos_ecef = norm(S.pos_fused_ecef(end,:) - pos_truth_ecef(end,:));
final_vel_ecef = norm(S.vel_fused_ecef(end,:) - vel_truth_ecef(end,:));
final_pos_body = norm(S.pos_fused_body(end,:) - pos_truth_body(end,:));
final_vel_body = norm(S.vel_fused_body(end,:) - vel_truth_body(end,:));

summary.frame = {'NED';'ECEF';'Body'};
summary.rmse_pos  = [rmse_pos_ned; rmse_pos_ecef; rmse_pos_body];
summary.final_pos = [final_pos_ned; final_pos_ecef; final_pos_body];
summary.rmse_vel  = [rmse_vel_ned; rmse_vel_ecef; rmse_vel_body];
summary.final_vel = [final_vel_ned; final_vel_ecef; final_vel_body];

save(fullfile(results_dir,[tag '_task6_stats.mat']),'summary');
fprintf('Task 6 overlay plots saved under: %s\n', results_dir);
end

function plot_overlay(time, pos_truth, vel_truth, pos_fused, vel_fused, title_str, out_base)
labels = {'X','Y','Z'};
colors = lines(3);
fig = figure('Visible','off');
axs = gobjects(2,3);
for i=1:3
    axs(1,i) = subplot(2,3,i); hold on; grid on;
    plot(time, pos_fused(:,i), 'Color', colors(i,:), 'LineWidth',1.8);
    plot(time, pos_truth(:,i),'--','Color',colors(i,:), 'LineWidth',1.2);
    ylabel(sprintf('Pos %s [m]', labels{i}));
    title(sprintf('Position %s', labels{i}));
    if i==1
        legend({'Fused','Truth'},'Location','best');
    end
end
for i=1:3
    axs(2,i) = subplot(2,3,i+3); hold on; grid on;
    plot(time, vel_fused(:,i), 'Color', colors(i,:), 'LineWidth',1.8);
    plot(time, vel_truth(:,i),'--','Color',colors(i,:), 'LineWidth',1.2);
    ylabel(sprintf('Vel %s [m/s]', labels{i}));
    title(sprintf('Velocity %s', labels{i}));
end
linkaxes(axs(:),'x');
for i=1:3
    xlabel(axs(2,i),'Time [s]');
end
sgtitle(title_str);
print(fig, [out_base '.pdf'], '-dpdf', '-bestfit');
print(fig, [out_base '.png'], '-dpng');
close(fig);
end

function [pos_ecef, vel_ecef] = ned2ecef_series(pos_ned, vel_ned, lat0, lon0, r0)
C_EN = compute_C_ECEF_to_NED(lat0, lon0);
C_NE = C_EN';
pos_ecef = (C_NE*pos_ned')' + r0.';
vel_ecef = (C_NE*vel_ned')';
end

function T = read_truth_file(fname)
    data = readmatrix(fname);
    T.time   = data(:,1);
    T.pos_N  = data(:,2);  T.pos_E = data(:,3);  T.pos_D = data(:,4);
    T.vel_N  = data(:,5);  T.vel_E = data(:,6);  T.vel_D = data(:,7);
end
