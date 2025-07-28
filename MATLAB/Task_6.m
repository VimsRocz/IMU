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
    'pos_fused_ned','vel_fused_ned', ...
    'pos_fused_ecef','vel_fused_ecef', ...
    'pos_fused_body','vel_fused_body');

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

%% Generate figures
out_dir = 'results';
if ~exist(out_dir,'dir'); mkdir(out_dir); end

plot_state_overlay('NED', est_t, S.pos_fused_ned, S.vel_fused_ned, ...
    pos_ned_i, vel_ned_i, dataset_tag, method, out_dir, tag);
plot_state_overlay('ECEF', est_t, S.pos_fused_ecef, S.vel_fused_ecef, ...
    pos_ecef_i, vel_ecef_i, dataset_tag, method, out_dir, tag);
plot_state_overlay('Body', est_t, S.pos_fused_body, S.vel_fused_body, ...
    pos_body_i, vel_body_i, dataset_tag, method, out_dir, tag);
end

%% --------------------------------------------------------------------
function plot_state_overlay(frame, t, pos_fused, vel_fused, pos_truth, vel_truth, dataset_tag, method, out_dir, tag)
labels = {'X','Y','Z'};
if strcmpi(frame,'NED')
    labels = {'N','E','D'};
end
f = figure('Visible','off');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
ax1 = gobjects(1,3); ax2 = gobjects(1,3);
for i=1:3
    ax1(i) = nexttile(i); hold on;
    plot(t, pos_fused(:,i), 'b-', 'LineWidth',1.0);
    plot(t, pos_truth(:,i), 'r--', 'LineWidth',1.0);
    ylabel('Position (m)'); title(sprintf('Position %s', labels{i}));
    grid on; axis tight;
    if i==3
        legend({'Fused','Truth'},'Location','northeast');
    end
end
for i=1:3
    ax2(i) = nexttile(i+3); hold on;
    plot(t, vel_fused(:,i), 'b-', 'LineWidth',1.0);
    plot(t, vel_truth(:,i), 'r--', 'LineWidth',1.0);
    ylabel('Velocity (m/s)'); title(sprintf('Velocity %s', labels{i}));
    grid on; axis tight;
end
linkaxes(ax1,'y');
linkaxes(ax2,'y');
xlabel(ax2(1),'Epoch Time (s)');
xlabel(ax2(2),'Epoch Time (s)');
xlabel(ax2(3),'Epoch Time (s)');
sgtitle(sprintf('Task 6 \x2013 %s Overlay  (Dataset %s | Method %s)', frame, dataset_tag, method));

base = fullfile(out_dir, sprintf('%s_task6_overlay_state_%s', tag, frame));
print(f, [base '.pdf'], '-dpdf', '-bestfit');
print(f, [base '.png'], '-dpng');
close(f);
fprintf('Task 6: saved overlay %s figure  ->  %s.pdf\n', frame, base);
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

