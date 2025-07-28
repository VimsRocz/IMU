function task7_error_analysis_plots(task6_file, output_dir)
%TASK7_ERROR_ANALYSIS_PLOTS  Error analysis and difference plots for Task 7.
%   TASK7_ERROR_ANALYSIS_PLOTS(TASK6_FILE, OUTPUT_DIR) loads the downsampled
%   fused state and truth data saved by Task 6. Residuals for position and
%   velocity are computed in the NED frame along with error norms. The
%   function also plots attitude angles and truth minus fused differences in
%   ECEF, NED and body frames. Figures are saved under OUTPUT_DIR.
%
%   This mirrors the prototype MATLAB function provided by the repository
%   user and complements ``task7_basic_residuals_plot.py`` in Python.
%
%   Example:
%       task7_error_analysis_plots('IMU_X002_GNSS_X002_TRIAD_task6_results.mat', get_results_dir())
%
%   See also TASK6_TRUTH_OVERLAY, TASK7_BASIC_RESIDUALS_PLOT.

if nargin < 2 || isempty(output_dir)
    output_dir = get_results_dir();
end
if ~exist(output_dir, 'dir'); mkdir(output_dir); end

S = load(fullfile(output_dir, task6_file), ...
    'pos_est_ned_ds', 'vel_est_ned_ds', 'pos_truth_ned', 'vel_truth_ned', ...
    'attitude_est_ds', 'acc_body_ds', 'acc_truth_body', ...
    'pos_est_ecef_ds', 'vel_est_ecef_ds', 'pos_truth_ecef', 'vel_truth_ecef', ...
    'time_indices');

pos_residuals_ned = S.pos_est_ned_ds - S.pos_truth_ned;
vel_residuals_ned = S.vel_est_ned_ds - S.vel_truth_ned;

pos_error_norm = sqrt(sum((S.pos_est_ned_ds - S.pos_truth_ned).^2, 1));
vel_error_norm = sqrt(sum((S.vel_est_ned_ds - S.vel_truth_ned).^2, 1));
att_error_norm = sqrt(sum((S.attitude_est_ds).^2, 1));

diff_ecef = S.pos_est_ecef_ds - S.pos_truth_ecef;
diff_ned  = S.pos_est_ned_ds  - S.pos_truth_ned;
diff_body = S.acc_body_ds     - S.acc_truth_body;

time_idx = S.time_indices;

% Subtask 7.3.1: residual plots
fig = figure('Visible','off');
subplot(3,2,1); plot(time_idx, pos_residuals_ned(1,:), 'b'); title('North Position Residual'); ylabel('m'); grid on;
subplot(3,2,2); plot(time_idx, pos_residuals_ned(2,:), 'g'); title('East Position Residual');  ylabel('m'); grid on;
subplot(3,2,3); plot(time_idx, pos_residuals_ned(3,:), 'k'); title('Down Position Residual');  ylabel('m'); grid on;
subplot(3,2,4); plot(time_idx, vel_residuals_ned(1,:), 'b'); title('North Velocity Residual'); ylabel('m/s'); grid on;
subplot(3,2,5); plot(time_idx, vel_residuals_ned(2,:), 'g'); title('East Velocity Residual');  ylabel('m/s'); grid on;
subplot(3,2,6); plot(time_idx, vel_residuals_ned(3,:), 'k'); title('Down Velocity Residual'); ylabel('m/s'); grid on;
res_pdf = fullfile(output_dir, 'task7_3_residuals_position_velocity.pdf');
print(fig, res_pdf, '-dpdf', '-bestfit');
print(fig, replace(res_pdf,'.pdf','.png'), '-dpng');
close(fig);

% Subtask 7.3.2: error norms
fig = figure('Visible','off');
subplot(3,1,1); plot(time_idx, pos_error_norm, 'b'); title('Position Error Norm'); ylabel('m'); grid on;
subplot(3,1,2); plot(time_idx, vel_error_norm, 'g'); title('Velocity Error Norm'); ylabel('m/s'); grid on;
subplot(3,1,3); plot(time_idx, att_error_norm, 'k'); title('Attitude Error Norm'); ylabel('rad'); grid on;
err_pdf = fullfile(output_dir, 'task7_3_error_norms.pdf');
print(fig, err_pdf, '-dpdf', '-bestfit');
print(fig, replace(err_pdf,'.pdf','.png'), '-dpng');
close(fig);

% Subtask 7.4: attitude angles
fig = figure('Visible','off');
subplot(3,1,1); plot(time_idx, S.attitude_est_ds(1,:), 'b'); title('Roll Angle');  ylabel('rad'); grid on;
subplot(3,1,2); plot(time_idx, S.attitude_est_ds(2,:), 'g'); title('Pitch Angle'); ylabel('rad'); grid on;
subplot(3,1,3); plot(time_idx, S.attitude_est_ds(3,:), 'k'); title('Yaw Angle');   ylabel('rad'); grid on;
att_pdf = fullfile(output_dir, 'task7_4_attitude_angles_euler.pdf');
print(fig, att_pdf, '-dpdf', '-bestfit');
print(fig, replace(att_pdf,'.pdf','.png'), '-dpng');
close(fig);

% Subtask 7.5: differences
frames = {'ECEF','NED','Body'};
diffs = {diff_ecef, diff_ned, diff_body};
for i=1:3
    fig = figure('Visible','off');
    subplot(3,1,1); plot(time_idx, diffs{i}(1,:), 'b'); ylabel('X [m]'); grid on;
    subplot(3,1,2); plot(time_idx, diffs{i}(2,:), 'g'); ylabel('Y [m]'); grid on;
    subplot(3,1,3); plot(time_idx, diffs{i}(3,:), 'k'); ylabel('Z [m]'); grid on; xlabel('Sample');
    sgtitle(['Truth - Fused Difference ' frames{i}]);
    base = fullfile(output_dir, ['task7_5_diff_truth_fused_over_time_' frames{i}]);
    print(fig, [base '.pdf'], '-dpdf', '-bestfit');
    print(fig, [base '.png'], '-dpng');
    close(fig);
end

save(fullfile(output_dir, 'task7_results.mat'), ...
    'pos_residuals_ned','vel_residuals_ned','pos_error_norm','vel_error_norm', ...
    'att_error_norm','diff_ecef','diff_ned','diff_body');
end
