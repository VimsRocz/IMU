function evaluate_filter_results(npz_file, output_dir, tag)
%EVALUATE_FILTER_RESULTS  Analyse filter residuals saved in NPZ format.
%
%   evaluate_filter_results(npz_file, output_dir, tag) mirrors the Python
%   ``run_evaluation_npz`` function. Residual position and velocity as well as
%   quaternion attitude history are loaded from ``npz_file`` and basic
%   statistics and plots are produced under ``output_dir``. ``tag`` is an
%   optional prefix for the output filenames. The routine is divided into
%   Subtasks 7.1-7.5 for clearer debugging output.

if nargin < 2 || isempty(output_dir)
    output_dir = 'output_matlab';
end
if nargin < 3
    tag = '';
end

if ~exist(output_dir, 'dir'); mkdir(output_dir); end
prefix = '';
if ~isempty(tag); prefix = [tag '_']; end

fprintf('--- Task 7, Subtask 7.1: Loading data and checking dimensions ---\n');

data = py.numpy.load(string(npz_file));
res_pos = double(data{'residual_pos'});
res_vel = double(data{'residual_vel'});
t = double(data{'time_residuals'});
quat = double(data{'attitude_q'});

n = min([size(res_pos,1), size(res_vel,1), numel(t), size(quat,1)]);
res_pos = res_pos(1:n,:);
res_vel = res_vel(1:n,:);
t = t(1:n);
quat = quat(1:n,:);

fprintf('Loaded %d samples. res_pos size %s, res_vel size %s\n', n, mat2str(size(res_pos)), mat2str(size(res_vel)));

fprintf('--- Task 7, Subtask 7.2: Computing residuals ---\n');

if isKey(data, 'time') && isKey(data,'pos_ned') && isKey(data,'vel_ned')
    fused_t = double(data{'time'});
    fused_pos = double(data{'pos_ned'});
    fused_vel = double(data{'vel_ned'});
    pos_interp = interp1(fused_t, fused_pos, t, 'linear', 'extrap');
    vel_interp = interp1(fused_t, fused_vel, t, 'linear', 'extrap');
    truth_pos = pos_interp - res_pos;
    truth_vel = derive_velocity(t, truth_pos);
    res_pos = pos_interp - truth_pos;
    res_vel = vel_interp - truth_vel;
else
    fused_t = t;
    pos_interp = nan(size(res_pos));
    vel_interp = nan(size(res_vel));
    truth_pos = nan(size(res_pos));
    truth_vel = nan(size(res_vel));
end

mean_pos = mean(res_pos, 1);
std_pos = std(res_pos, [], 1);
mean_vel = mean(res_vel, 1);
std_vel = std(res_vel, [], 1);
fprintf('Position residual mean [m]: %s\n', mat2str(mean_pos,3));
fprintf('Position residual std  [m]: %s\n', mat2str(std_pos,3));
fprintf('Velocity residual mean [m/s]: %s\n', mat2str(mean_vel,3));
fprintf('Velocity residual std  [m/s]: %s\n', mat2str(std_vel,3));

fprintf('--- Task 7, Subtask 7.3: Saving residual and norm plots ---\n');

labels = {'X','Y','Z'};
f = figure('Visible','off','Position',[100 100 900 450]);
for i = 1:3
    subplot(2,3,i); plot(t, res_pos(:,i)); title(labels{i}); ylabel('Pos Residual [m]'); grid on;
    subplot(2,3,i+3); plot(t, res_vel(:,i)); xlabel('Time [s]'); ylabel('Vel Residual [m/s]'); grid on;
end
sgtitle('Task 7 - GNSS - Predicted Residuals');
set(f,'PaperPositionMode','auto');
pdf = fullfile(output_dir, sprintf('%stask7_3_residuals_position_velocity.pdf', prefix));
print(f, pdf, '-dpdf', '-bestfit');
close(f); fprintf('Saved %s\n', pdf);

fprintf('--- Task 7, Subtask 7.4: Plotting attitude angles ---\n');
eul = rad2deg(quat2eul(quat(:,[2 3 4 1])));
f = figure('Visible','off','Position',[100 100 600 500]);
names = {'Roll','Pitch','Yaw'};
for i = 1:3
    subplot(3,1,i); plot(t, eul(:,i)); ylabel([names{i} ' [deg]']); grid on;
end
xlabel('Time [s]'); sgtitle('Task 7 - Attitude Angles');
set(f,'PaperPositionMode','auto');
pdf_att = fullfile(output_dir, sprintf('%stask7_4_attitude_angles_euler.pdf', prefix));
print(f, pdf_att, '-dpdf', '-bestfit'); close(f); fprintf('Saved %s\n', pdf_att);

norm_pos = vecnorm(res_pos,2,2);
norm_vel = vecnorm(res_vel,2,2);
res_acc = gradient(res_vel, t);
norm_acc = vecnorm(res_acc,2,2);

f = figure('Visible','off','Position',[100 100 600 400]);
plot(t, norm_pos, 'DisplayName','|pos error|'); hold on;
plot(t, norm_vel, 'DisplayName','|vel error|');
plot(t, norm_acc, 'DisplayName','|acc error|');
xlabel('Time [s]'); ylabel('Error Norm'); legend; grid on;
set(f,'PaperPositionMode','auto');
pdf_norm = fullfile(output_dir, sprintf('%stask7_3_error_norms.pdf', prefix));
print(f, pdf_norm, '-dpdf', '-bestfit'); close(f); fprintf('Saved %s\n', pdf_norm);

% Subtask 7.5: difference truth - fused over time
fprintf('--- Task 7, Subtask 7.5: Plotting Truth - Fused differences ---\n');
if ~any(isnan(truth_pos(:))) && ~any(isnan(pos_interp(:)))
    run_id = strrep(tag, filesep, '_');
    if isempty(run_id); run_id = 'run'; end
    out_dir = fullfile('output_matlab');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    subtask7_5_diff_plot(t, pos_interp, truth_pos, vel_interp, truth_vel, run_id, out_dir);
else
    fprintf('Truth or fused data missing, skipping Subtask 7.5.\n');
end

rmse_pos = sqrt(mean(norm_pos.^2));
rmse_vel = sqrt(mean(norm_vel.^2));
rmse_acc = sqrt(mean(norm_acc.^2));
final_pos = norm_pos(end);
final_vel = norm_vel(end);
final_acc = norm_acc(end);

fprintf('[SUMMARY] rmse_pos=%.3f m final_pos=%.3f m rmse_vel=%.3f m/s final_vel=%.3f m/s\n', ...
    rmse_pos, final_pos, rmse_vel, final_vel);

end


function [diff_pos_ned, diff_vel_ned] = subtask7_5_diff_plot(time, fused_pos_ned, truth_pos_ned, fused_vel_ned, truth_vel_ned, run_id, out_dir)
%SUBTASK7_5_DIFF_PLOT Plot and analyse Truth minus Fused differences in NED frame.
%   [diff_pos_ned, diff_vel_ned] = SUBTASK7_5_DIFF_PLOT(time, fused_pos_ned,
%   truth_pos_ned, fused_vel_ned, truth_vel_ned, run_id, out_dir) computes
%   truth_pos_ned - fused_pos_ned and truth_vel_ned - fused_vel_ned at each
%   time step and plots the results in the NED frame.

diff_pos_ned = truth_pos_ned - fused_pos_ned;
diff_vel_ned = truth_vel_ned - fused_vel_ned;

labels = {'North','East','Down'};
f = figure('Visible','off','Position',[100 100 900 450]);
for i = 1:3
    subplot(2,3,i); plot(time, diff_pos_ned(:,i));
    title(labels{i}); ylabel('Difference [m]'); grid on;
end
for i = 1:3
    subplot(2,3,3+i); plot(time, diff_vel_ned(:,i));
    xlabel('Time [s]'); ylabel('Difference [m/s]'); grid on;
end
sgtitle('Truth - Fused Differences (NED Frame)');
set(f,'PaperPositionMode','auto');
out_file_pdf = fullfile(out_dir, [run_id '_task7_5_diff_truth_fused_over_time.pdf']);
out_file_png = strrep(out_file_pdf, '.pdf', '.png');
print(f, out_file_pdf, '-dpdf', '-bestfit');
print(f, out_file_png, '-dpng');
close(f); fprintf('Saved %s\n', out_file_pdf);

pos_thr = 1.0; vel_thr = 1.0;
for i = 1:3
    dp = diff_pos_ned(:,i); dv = diff_vel_ned(:,i);
    fprintf('%s position difference range: %.2f m to %.2f m. ', labels{i}, min(dp), max(dp));
    idx_p = find(abs(dp) > pos_thr);
    if ~isempty(idx_p)
        fprintf('Outliers (>%.1fm) at samples: %s\n', pos_thr, mat2str(idx_p'));
    else
        fprintf('No outliers (>%.1fm).\n', pos_thr);
    end
    fprintf('%s velocity difference range: %.2f m/s to %.2f m/s. ', labels{i}, min(dv), max(dv));
    idx_v = find(abs(dv) > vel_thr);
    if ~isempty(idx_v)
        fprintf('Outliers (>%.1fm/s) at samples: %s\n', vel_thr, mat2str(idx_v'));
    else
        fprintf('No outliers (>%.1fm/s).\n', vel_thr);
    end
end
end
