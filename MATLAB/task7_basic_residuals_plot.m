function task7_basic_residuals_plot(task5_file, task4_file, run_id, out_dir)
%TASK7_BASIC_RESIDUALS_PLOT Simple residual analysis for Task 7.
%   TASK7_BASIC_RESIDUALS_PLOT(TASK5_FILE, TASK4_FILE, RUN_ID, OUT_DIR) loads
%   the fused state log ``x_log`` produced in Task 5 and GNSS truth data from
%   Task 4. Position and velocity residuals are computed in the NED frame
%   after downsampling the estimator output to match the GNSS sampling rate.
%   Residual statistics and plots are written to OUT_DIR with RUN_ID
%   embedded in the filenames.  This mirrors ``task7_ned_residuals_plot.py``.
%
%   Example:
%       task7_basic_residuals_plot('results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat', ...
%             'results/Task4_results_IMU_X002_GNSS_X002.mat', ...
%             'IMU_X002_GNSS_X002_TRIAD', 'results');

if nargin < 4 || isempty(out_dir)
    out_dir = get_results_dir();
end
if ~exist(out_dir,'dir'); mkdir(out_dir); end

load(task5_file, 'x_log');
load(task4_file, 'pos_ned_gnss', 'vel_ned_gnss');

pos_est = x_log(1:3, :);
vel_est = x_log(4:6, :);

% Downsample to match GNSS (1250 samples)
N_est = size(pos_est, 2);
N_truth = size(pos_ned_gnss, 1);
downsample_factor = floor(N_est / N_truth);
time_indices = 1:downsample_factor:N_est;

pos_est = pos_est(:, time_indices);
vel_est = vel_est(:, time_indices);

pos_truth = pos_ned_gnss';
vel_truth = vel_ned_gnss';

if size(pos_truth,2) ~= numel(time_indices)
    error('Truth and estimated data lengths do not match.');
end

pos_residuals = pos_est - pos_truth;
vel_residuals = vel_est - vel_truth;

pos_res_mean = mean(pos_residuals, 2);
pos_res_std = std(pos_residuals, 0, 2);
vel_res_mean = mean(vel_residuals, 2);
vel_res_std = std(vel_residuals, 0, 2);

fprintf('Position residual mean [m]: [%.8f %.8f %.8f]\n', pos_res_mean);
fprintf('Position residual std  [m]: [%.8f %.8f %.8f]\n', pos_res_std);
fprintf('Velocity residual mean [m/s]: [%.8f %.8f %.8f]\n', vel_res_mean);
fprintf('Velocity residual std  [m/s]: [%.8f %.8f %.8f]\n', vel_res_std);

fig = figure('Visible','off');
subplot(2,1,1);
plot(pos_residuals(1,:), 'b', 'DisplayName','North'); hold on;
plot(pos_residuals(2,:), 'g', 'DisplayName','East');
plot(pos_residuals(3,:), 'k', 'DisplayName','Down');
legend('Location','best'); grid on;
xlabel('Sample'); ylabel('Residual (m)');
title('Position Residuals (NED)');

subplot(2,1,2);
plot(vel_residuals(1,:), 'b', 'DisplayName','North'); hold on;
plot(vel_residuals(2,:), 'g', 'DisplayName','East');
plot(vel_residuals(3,:), 'k', 'DisplayName','Down');
legend('Location','best'); grid on;
xlabel('Sample'); ylabel('Residual (m/s)');
title('Velocity Residuals (NED)');

base = fullfile(out_dir, sprintf('%s_task7_3_residuals_position_velocity', run_id));
print(fig, [base '.pdf'], '-dpdf', '-bestfit');
print(fig, [base '.png'], '-dpng', '-bestfit');
close(fig);
fprintf('Saved %s.pdf\n', base);

labels = {'North','East','Down'};
for i = 1:3
    pos_range = [min(pos_residuals(i,:)), max(pos_residuals(i,:))];
    vel_range = [min(vel_residuals(i,:)), max(vel_residuals(i,:))];
    pos_exceed = sum(abs(pos_residuals(i,:)) > 1);
    vel_exceed = sum(abs(vel_residuals(i,:)) > 1);
    fprintf('NED %s position diff range: %.2f m to %.2f m. %d samples exceed 1.0 m\n', ...
        labels{i}, pos_range(1), pos_range(2), pos_exceed);
    fprintf('NED %s velocity diff range: %.2f m/s to %.2f m/s. %d samples exceed 1.0 m/s\n', ...
        labels{i}, vel_range(1), vel_range(2), vel_exceed);
end
end
