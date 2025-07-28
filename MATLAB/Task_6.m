function Task_6(dataset_tag, method)
%TASK_6 Generate 3x3 overlay plots using STATE\_X001.txt.
%   TASK_6() loads the fused state history from Task 5 together with the
%   ground truth trajectory and visualises position, velocity and
%   acceleration in the ECEF, NED and body frames. Attitude angles stored in
%   ``x_log`` are also plotted. Results are written to the ``MATLAB/results``
%   folder following the standard naming convention used by the Python
%   pipeline.
%
%   This mirrors the functionality of ``task6_overlay_plot.py`` but includes
%   additional body frame and attitude visualisations.
%
%   Usage:
%       Task_6()
%
%   See also RUN_TRIAD_ONLY, TASK6_OVERLAY_PLOT.PY

if nargin < 1 || isempty(dataset_tag)
    dataset_tag = 'IMU_X002_GNSS_X002';
end
if nargin < 2 || isempty(method)
    method = 'TRIAD';
end

fprintf('--- Starting Task 6: Overlay Plots with STATE_X001.txt ---\n');

%% ------------------------------------------------------------------
% Configuration
%% ------------------------------------------------------------------
root_dir    = fileparts(fileparts(mfilename('fullpath')));
results_dir = get_results_dir();
run_id    = sprintf('%s_%s', dataset_tag, method);

results_file = fullfile(results_dir, sprintf('Task5_%s_%s.mat', dataset_tag, method));
truth_file   = fullfile(root_dir, 'STATE_X001.txt');
task4_file   = fullfile(results_dir, sprintf('Task4_%s_%s.mat', dataset_tag, method));
task3_file   = fullfile(results_dir, sprintf('Task3_%s_%s.mat', dataset_tag, method));

%% ------------------------------------------------------------------
% Load estimator data from Task 5
%% ------------------------------------------------------------------
try
    load(results_file, 'x_log', 'pos_est_ecef', 'vel_est_ecef', ...
        'pos_est_ned', 'vel_est_ned', 'acc_body');
    fprintf('Task 6: Loaded %s (x\\_log size %dx%d)\n', results_file, ...
        size(x_log,1), size(x_log,2));
catch ME
    error('Task 6: Failed to load data from %s (%s).', results_file, ME.message);
end

%% ------------------------------------------------------------------
% Load ground truth
%% ------------------------------------------------------------------
try
    truth_data = readmatrix(truth_file);
    fprintf('Task 6: Loaded truth data from %s, size: %dx%d\n', truth_file, ...
        size(truth_data,1), size(truth_data,2));
catch ME
    error('Task 6: Failed to load truth data from %s (%s).', truth_file, ME.message);
end

% Extract ECEF truth position and velocity
pos_truth_ecef = truth_data(:, 2:4)';
vel_truth_ecef = truth_data(:, 5:7)';

%% ------------------------------------------------------------------
% Load rotation matrices from previous tasks
%% ------------------------------------------------------------------
try
    S4 = load(task4_file, 'C_n_e', 'gnss_ned_pos');
    C_n_e = S4.C_n_e;
    fprintf('Task 6: Loaded C_n_e and gnss_ned_pos from %s\n', task4_file);
catch ME
    error('Task 6: Failed to load data from %s (%s).', task4_file, ME.message);
end

try
    S3 = load(task3_file);
    if isfield(S3, 'task3_results') && isfield(S3.task3_results, method)
        C_b_n = S3.task3_results.(method).R;
    elseif isfield(S3, 'C_b2n')
        C_b_n = S3.C_b2n;
    else
        error('Rotation matrix not found in %s.', task3_file);
    end
    fprintf('Task 6: Loaded C_b_n from %s\n', task3_file);
catch ME
    error('Task 6: Failed to load rotation matrices (%s).', ME.message);
end

C_e_n = C_n_e'; % ECEF to NED

%% ------------------------------------------------------------------
% Convert truth data to NED and body frames
%% ------------------------------------------------------------------
pos_truth_ned = C_e_n * pos_truth_ecef;
vel_truth_ned = C_e_n * vel_truth_ecef;

% Body-frame acceleration placeholder (truth acceleration not provided)
acc_truth_body = zeros(3, size(pos_truth_ecef,2));
for t = 1:size(acc_truth_body,2)
    if ndims(C_b_n) == 3
        acc_truth_body(:,t) = C_b_n(:,:,min(t,end)) * C_e_n * [0;0;0];
    else
        acc_truth_body(:,t) = C_b_n * C_e_n * [0;0;0];
    end
end

%% ------------------------------------------------------------------
% Downsample estimates to match truth length
%% ------------------------------------------------------------------
downsample_factor = 400; % 500000 / 1250
idx = 1:downsample_factor:500000;

pos_est_ecef_ds = pos_est_ecef(:, idx);
vel_est_ecef_ds = vel_est_ecef(:, idx);
pos_est_ned_ds  = pos_est_ned(:, idx);
vel_est_ned_ds  = vel_est_ned(:, idx);
acc_body_ds     = acc_body(:, idx);

if size(pos_truth_ecef,2) ~= numel(idx)
    error('Task 6: Data length mismatch. Truth %d vs Est %d samples.', ...
        size(pos_truth_ecef,2), numel(idx));
end
fprintf('Task 6: Downsampled estimates to %d samples (factor %d)\n', ...
    numel(idx), downsample_factor);

attitude_est = x_log(7:9, idx); % Euler angles [roll; pitch; yaw]

fprintf('Subtask 6.8.2: Plotted TRIAD position X\_ECEF: First = %.4f, Last = %.4f m\n', ...
    pos_est_ecef_ds(1,1), pos_est_ecef_ds(1,end));
fprintf('Subtask 6.8.2: Plotted TRIAD position North\_NED: First = %.4f, Last = %.4f m\n', ...
    pos_est_ned_ds(1,1), pos_est_ned_ds(1,end));

%% ------------------------------------------------------------------
% Plotting helpers
%% ------------------------------------------------------------------
run_tag = run_id;

plot_ecef(run_tag, idx, pos_est_ecef_ds, vel_est_ecef_ds, ...
    pos_truth_ecef, vel_truth_ecef, results_dir);
plot_ned(run_tag, idx, pos_est_ned_ds, vel_est_ned_ds, ...
    pos_truth_ned, vel_truth_ned, results_dir);
plot_body(run_tag, idx, acc_body_ds, acc_truth_body, results_dir);
plot_attitude(run_tag, idx, attitude_est, results_dir);

% Save results MAT-file
out_mat = fullfile(results_dir, sprintf('%s_task6_results.mat', run_tag));
save(out_mat, 'pos_est_ecef_ds', 'vel_est_ecef_ds', 'pos_est_ned_ds', ...
    'vel_est_ned_ds', 'acc_body_ds', 'pos_truth_ecef', 'vel_truth_ecef', ...
    'pos_truth_ned', 'vel_truth_ned', 'acc_truth_body', 'attitude_est');
fprintf('Task 6: Results saved to %s\n', out_mat);
fprintf('Task 6: Completed successfully\n');
end

%% ========================================================================
% Local plotting functions
%% ========================================================================
function plot_ecef(tag, t, pos_e, vel_e, pos_t, vel_t, out_dir)
    fprintf('Task 6: Generating 3x3 ECEF overlay plot...\n');
    fig = figure('Name', 'Task 6 - ECEF Overlay (3x3)', 'Visible', 'off');
    subplot(3,3,1); plot(t, pos_e(1,:), 'b'); hold on; plot(t, pos_t(1,:), 'r--');
    title('Position X'); ylabel('m'); legend('Est','Truth'); grid on;
    subplot(3,3,2); plot(t, pos_e(2,:), 'g'); hold on; plot(t, pos_t(2,:), 'm--');
    title('Position Y'); ylabel('m'); legend('Est','Truth'); grid on;
    subplot(3,3,3); plot(t, pos_e(3,:), 'k'); hold on; plot(t, pos_t(3,:), 'c--');
    title('Position Z'); ylabel('m'); legend('Est','Truth'); grid on;
    subplot(3,3,4); plot(t, vel_e(1,:), 'b'); hold on; plot(t, vel_t(1,:), 'r--');
    title('Velocity X'); ylabel('m/s'); legend('Est','Truth'); grid on;
    subplot(3,3,5); plot(t, vel_e(2,:), 'g'); hold on; plot(t, vel_t(2,:), 'm--');
    title('Velocity Y'); ylabel('m/s'); legend('Est','Truth'); grid on;
    subplot(3,3,6); plot(t, vel_e(3,:), 'k'); hold on; plot(t, vel_t(3,:), 'c--');
    title('Velocity Z'); ylabel('m/s'); legend('Est','Truth'); grid on;
    subplot(3,3,7); plot(t, sqrt(sum(pos_e.^2,1)), 'b'); hold on;
    plot(t, sqrt(sum(pos_t.^2,1)), 'r--'); title('Position Norm'); ylabel('m');
    legend('Est','Truth'); grid on;
    subplot(3,3,8); plot(t, sqrt(sum(vel_e.^2,1)), 'b'); hold on;
    plot(t, sqrt(sum(vel_t.^2,1)), 'r--'); title('Velocity Norm'); ylabel('m/s');
    legend('Est','Truth'); grid on;
    subplot(3,3,9); plot(t, sqrt(sum((pos_e - pos_t).^2,1)), 'k');
    title('Position Error Norm'); ylabel('m'); grid on;

    pdf = fullfile(out_dir, sprintf('%s_task6_overlay_state_ECEF.pdf', tag));
    png = strrep(pdf, '.pdf', '.png');
    print(fig, pdf, '-dpdf', '-bestfit');
    exportgraphics(fig, png, 'Resolution',300);
    close(fig);
    fprintf('Task 6: Saved plot: %s\n', pdf);
end

function plot_ned(tag, t, pos_n, vel_n, pos_t, vel_t, out_dir)
    fprintf('Task 6: Generating 3x3 NED overlay plot...\n');
    fig = figure('Name', 'Task 6 - NED Overlay (3x3)', 'Visible', 'off');
    subplot(3,3,1); plot(t, pos_n(1,:), 'b'); hold on; plot(t, pos_t(1,:), 'r--');
    title('Position North'); ylabel('m'); legend('Est','Truth'); grid on;
    subplot(3,3,2); plot(t, pos_n(2,:), 'g'); hold on; plot(t, pos_t(2,:), 'm--');
    title('Position East'); ylabel('m'); legend('Est','Truth'); grid on;
    subplot(3,3,3); plot(t, pos_n(3,:), 'k'); hold on; plot(t, pos_t(3,:), 'c--');
    title('Position Down'); ylabel('m'); legend('Est','Truth'); grid on;
    subplot(3,3,4); plot(t, vel_n(1,:), 'b'); hold on; plot(t, vel_t(1,:), 'r--');
    title('Velocity North'); ylabel('m/s'); legend('Est','Truth'); grid on;
    subplot(3,3,5); plot(t, vel_n(2,:), 'g'); hold on; plot(t, vel_t(2,:), 'm--');
    title('Velocity East'); ylabel('m/s'); legend('Est','Truth'); grid on;
    subplot(3,3,6); plot(t, vel_n(3,:), 'k'); hold on; plot(t, vel_t(3,:), 'c--');
    title('Velocity Down'); ylabel('m/s'); legend('Est','Truth'); grid on;
    subplot(3,3,7); plot(t, sqrt(sum(pos_n.^2,1)), 'b'); hold on;
    plot(t, sqrt(sum(pos_t.^2,1)), 'r--'); title('Position Norm'); ylabel('m');
    legend('Est','Truth'); grid on;
    subplot(3,3,8); plot(t, sqrt(sum(vel_n.^2,1)), 'b'); hold on;
    plot(t, sqrt(sum(vel_t.^2,1)), 'r--'); title('Velocity Norm'); ylabel('m/s');
    legend('Est','Truth'); grid on;
    subplot(3,3,9); plot(t, sqrt(sum((pos_n - pos_t).^2,1)), 'k');
    title('Position Error Norm'); ylabel('m'); grid on;

    pdf = fullfile(out_dir, sprintf('%s_task6_overlay_state_NED.pdf', tag));
    png = strrep(pdf, '.pdf', '.png');
    print(fig, pdf, '-dpdf', '-bestfit');
    exportgraphics(fig, png, 'Resolution',300);
    close(fig);
    fprintf('Task 6: Saved plot: %s\n', pdf);
end

function plot_body(tag, t, acc_b, acc_t, out_dir)
    fprintf('Task 6: Generating 3x3 Body overlay plot...\n');
    fig = figure('Name', 'Task 6 - Body Overlay (3x3)', 'Visible', 'off');
    subplot(3,3,1); plot(t, acc_b(1,:), 'b'); hold on; plot(t, acc_t(1,:), 'r--');
    title('Acceleration X'); ylabel('m/s^2'); legend('Est','Truth'); grid on;
    subplot(3,3,2); plot(t, acc_b(2,:), 'g'); hold on; plot(t, acc_t(2,:), 'm--');
    title('Acceleration Y'); ylabel('m/s^2'); legend('Est','Truth'); grid on;
    subplot(3,3,3); plot(t, acc_b(3,:), 'k'); hold on; plot(t, acc_t(3,:), 'c--');
    title('Acceleration Z'); ylabel('m/s^2'); legend('Est','Truth'); grid on;
    subplot(3,3,4); plot(t, zeros(size(t)), 'b'); title('Placeholder'); grid on;
    subplot(3,3,5); plot(t, zeros(size(t)), 'g'); title('Placeholder'); grid on;
    subplot(3,3,6); plot(t, zeros(size(t)), 'k'); title('Placeholder'); grid on;
    subplot(3,3,7); plot(t, sqrt(sum(acc_b.^2,1)), 'b'); hold on;
    plot(t, sqrt(sum(acc_t.^2,1)), 'r--'); title('Acceleration Norm'); ylabel('m/s^2'); legend('Est','Truth'); grid on;
    subplot(3,3,8); plot(t, zeros(size(t)), 'b'); title('Placeholder'); grid on;
    subplot(3,3,9); plot(t, sqrt(sum((acc_b - acc_t).^2,1)), 'k');
    title('Acceleration Error Norm'); ylabel('m/s^2'); grid on;

    pdf = fullfile(out_dir, sprintf('%s_task6_overlay_state_Body.pdf', tag));
    png = strrep(pdf, '.pdf', '.png');
    print(fig, pdf, '-dpdf', '-bestfit');
    exportgraphics(fig, png, 'Resolution',300);
    close(fig);
    fprintf('Task 6: Saved plot: %s\n', pdf);
end

function plot_attitude(tag, t, eul, out_dir)
    fprintf('Task 6: Generating 3x3 Attitude Angles plot...\n');
    fig = figure('Name', 'Task 6 - Attitude Angles (3x3)', 'Visible', 'off');
    subplot(3,3,1); plot(t, eul(1,:), 'b'); title('Roll Angle'); ylabel('rad'); grid on;
    subplot(3,3,2); plot(t, eul(2,:), 'g'); title('Pitch Angle'); ylabel('rad'); grid on;
    subplot(3,3,3); plot(t, eul(3,:), 'k'); title('Yaw Angle'); ylabel('rad'); grid on;
    subplot(3,3,4); plot(t, zeros(size(t)), 'b'); title('Placeholder'); grid on;
    subplot(3,3,5); plot(t, zeros(size(t)), 'g'); title('Placeholder'); grid on;
    subplot(3,3,6); plot(t, zeros(size(t)), 'k'); title('Placeholder'); grid on;
    subplot(3,3,7); plot(t, sqrt(sum(eul.^2,1)), 'b'); title('Attitude Norm'); ylabel('rad'); grid on;
    subplot(3,3,8); plot(t, zeros(size(t)), 'b'); title('Placeholder'); grid on;
    subplot(3,3,9); plot(t, zeros(size(t)), 'k'); title('Placeholder'); grid on;

    pdf = fullfile(out_dir, sprintf('%s_task6_attitude_angles.pdf', tag));
    png = strrep(pdf, '.pdf', '.png');
    print(fig, pdf, '-dpdf', '-bestfit');
    exportgraphics(fig, png, 'Resolution',300);
    close(fig);
    fprintf('Task 6: Saved plot: %s\n', pdf);
end
