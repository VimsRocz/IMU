function task6_truth_overlay(gnss_data, task3_results, task4_results, task5_results, results_dir, dt)
%TASK6_TRUTH_OVERLAY  Compare fused navigation solution with GNSS truth.
%
%   TASK6_TRUTH_OVERLAY(GNSS_DATA, TASK3_RESULTS, TASK4_RESULTS, TASK5_RESULTS,
%   RESULTS_DIR, DT) loads the Kalman filter results from Task 5 together
%   with the NED reference frame parameters from Task 4 and the initial body
%   attitude from Task 3. GNSS_DATA is a table or matrix containing the raw
%   GNSS measurements with a Posix_Time column. The function interpolates
%   the truth data to match the IMU sample interval DT, transforms positions
%   and velocities between NED, ECEF and body frames, then generates overlay
%   plots of fused versus truth states. Summary RMSE and final error metrics
%   are printed and saved alongside the plotted figures under RESULTS_DIR.
%
%   This mirrors the behaviour of the Python implementation used for Task 6.
%
%   Example:
%       task6_truth_overlay(gnss_tbl, 'Task3_results_IMU_X002_GNSS_X002.mat', ...
%           'Task4_results_IMU_X002_GNSS_X002.mat', ...
%           'IMU_X002_GNSS_X002_TRIAD_task5_results.mat', get_results_dir(), 0.0025);

if nargin < 6
    error('Not enough input arguments.');
end
if isempty(results_dir)
    results_dir = get_results_dir();
end
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
% Determine dataset and method from the Task 5 filename
[~,fname,~] = fileparts(task5_results);
parts = regexp(fname, '(IMU_\w+)_(GNSS_\w+)_([A-Za-z]+)_task5_results', 'tokens', 'once');
if ~isempty(parts)
    dataset = sprintf('%s_%s', parts{1}, parts{2});
    method  = parts{3};
else
    dataset = 'DATASET';
    method  = 'METHOD';
end

% Load previous task results
S5 = load(task5_results, 'x_log', 'pos_ned', 'vel_ned');
S4 = load(task4_results, 'pos_ned_gnss', 'vel_ned_gnss', 'C_e2n', 'C_n2e', 'r0');
S3 = load(task3_results, 'C_b2n');

pos_ned = S5.pos_ned;
vel_ned = S5.vel_ned;
x_log   = S5.x_log;

pos_ned_gnss = S4.pos_ned_gnss;
vel_ned_gnss = S4.vel_ned_gnss;
C_n2e = S4.C_n2e;
C_b2n = S3.C_b2n;
r0    = S4.r0(:); % column vector

N = size(pos_ned,1);
t_imu = (0:N-1).' * dt;

t_gnss = gnss_data.Posix_Time(:);
% Interpolate GNSS truth to IMU time base
pos_truth = interp1(t_gnss, pos_ned_gnss, t_imu, 'linear', 'extrap');
vel_truth = interp1(t_gnss, vel_ned_gnss, t_imu, 'linear', 'extrap');

% Transform fused and truth to ECEF
pos_ecef_fused = (C_n2e * pos_ned.' + r0).';
vel_ecef_fused = (C_n2e * vel_ned.').';
pos_ecef_truth = (C_n2e * pos_truth.' + r0).';
vel_ecef_truth = (C_n2e * vel_truth.').';

% Transform fused to body frame
pos_body_fused = (C_b2n.' * pos_ned.').';
vel_body_fused = (C_b2n.' * vel_ned.').';

% Attitude angles from state log (assume [roll; pitch; yaw] in radians)
if size(x_log,1) >= 9
    eul = rad2deg(x_log(7:9,:)).';
else
    eul = zeros(N,3);
end

% Compute RMSE and final errors in each frame
rmse_pos_ned  = sqrt(mean(sum((pos_ned - pos_truth).^2, 2)));
rmse_vel_ned  = sqrt(mean(sum((vel_ned - vel_truth).^2, 2)));
rmse_pos_ecef = sqrt(mean(sum((pos_ecef_fused - pos_ecef_truth).^2, 2)));
rmse_vel_ecef = sqrt(mean(sum((vel_ecef_fused - vel_ecef_truth).^2, 2)));
rmse_pos_body = sqrt(mean(sum(pos_body_fused.^2, 2)));
rmse_vel_body = sqrt(mean(sum(vel_body_fused.^2, 2)));

final_pos_ned  = norm(pos_ned(end,:)  - pos_truth(end,:));
final_vel_ned  = norm(vel_ned(end,:)  - vel_truth(end,:));
final_pos_ecef = norm(pos_ecef_fused(end,:) - pos_ecef_truth(end,:));
final_vel_ecef = norm(vel_ecef_fused(end,:) - vel_ecef_truth(end,:));
final_pos_body = norm(pos_body_fused(end,:));
final_vel_body = norm(vel_body_fused(end,:));

%------------------------------------------------------------------
% Overlay plots
%------------------------------------------------------------------
labels_ned  = {'N','E','D'};
labels_ecef = {'X','Y','Z'};

run_id = sprintf('%s_%s', dataset, method);
f_ned = figure('Visible','off');
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_imu, pos_ned(:,i), 'r', 'DisplayName','Fused');
    plot(t_imu, pos_truth(:,i), 'b--', 'DisplayName','Truth');
    ylabel(['Pos ' labels_ned{i} ' [m]']); grid on;
    if i==1; legend('Location','best'); title('Position NED'); end
end
xlabel('Time [s]');
set(f_ned,'PaperPositionMode','auto');

pdf_ned = fullfile(results_dir, sprintf('%s_task6_overlay_state_NED.pdf', run_id));
png_ned = strrep(pdf_ned,'.pdf','.png');
savefig(gcf); % replaced for interactive
savefig(gcf); % replaced for interactive
close(f_ned);

f_ecef = figure('Visible','off');
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_imu, pos_ecef_fused(:,i), 'r', 'DisplayName','Fused');
    plot(t_imu, pos_ecef_truth(:,i), 'b--', 'DisplayName','Truth');
    ylabel(['Pos ' labels_ecef{i} ' [m]']); grid on;
    if i==1; legend('Location','best'); title('Position ECEF'); end
end
xlabel('Time [s]');
set(f_ecef,'PaperPositionMode','auto');

pdf_ecef = fullfile(results_dir, sprintf('%s_task6_overlay_state_ECEF.pdf', run_id));
png_ecef = strrep(pdf_ecef,'.pdf','.png');
savefig(gcf); % replaced for interactive
savefig(gcf); % replaced for interactive
close(f_ecef);

f_body = figure('Visible','off');
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_imu, pos_body_fused(:,i));
    ylabel(['Pos ' labels_ecef{i} ' [m]']); grid on;
    if i==1; title('Position Body'); end
end
xlabel('Time [s]');
set(f_body,'PaperPositionMode','auto');

pdf_body = fullfile(results_dir, sprintf('%s_task6_overlay_state_Body.pdf', run_id));
png_body = strrep(pdf_body,'.pdf','.png');
savefig(gcf); % replaced for interactive
savefig(gcf); % replaced for interactive
close(f_body);

f_att = figure('Visible','off');
plot(t_imu, eul);
legend({'Roll','Pitch','Yaw'}); grid on;
xlabel('Time [s]'); ylabel('Angle [deg]');
set(f_att,'PaperPositionMode','auto');

pdf_att = fullfile(results_dir, sprintf('%s_task6_attitude_angles.pdf', run_id));
png_att = strrep(pdf_att,'.pdf','.png');
savefig(gcf); % replaced for interactive
savefig(gcf); % replaced for interactive
close(f_att);

% Summary table
fprintf('Frame\tRMSEpos\tFinalPos\tRMSEvel\tFinalVel\n');
fprintf('NED\t%.3f\t%.3f\t%.3f\t%.3f\n', rmse_pos_ned, final_pos_ned, rmse_vel_ned, final_vel_ned);
fprintf('ECEF\t%.3f\t%.3f\t%.3f\t%.3f\n', rmse_pos_ecef, final_pos_ecef, rmse_vel_ecef, final_vel_ecef);
fprintf('Body\t%.3f\t%.3f\t%.3f\t%.3f\n', rmse_pos_body, final_pos_body, rmse_vel_body, final_vel_body);

save(fullfile(results_dir, sprintf('%s_task6_results.mat', run_id)), ...
    'pos_ned', 'vel_ned', 'pos_ecef_fused', 'vel_ecef_fused', ...
    'pos_body_fused', 'vel_body_fused', 'rmse_pos_ned', 'rmse_vel_ned', ...
    'rmse_pos_ecef', 'rmse_vel_ecef', 'rmse_pos_body', 'rmse_vel_body');
end
