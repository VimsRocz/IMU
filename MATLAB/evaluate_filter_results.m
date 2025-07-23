function evaluate_filter_results(npz_file, output_dir, tag)
%EVALUATE_FILTER_RESULTS  Analyse filter residuals saved in NPZ format.
%
%   evaluate_filter_results(npz_file, output_dir, tag) mirrors the Python
%   ``run_evaluation_npz`` function. Residual position and velocity as well as
%   quaternion attitude history are loaded from ``npz_file`` and basic
%   statistics and plots are produced under ``output_dir``. ``tag`` is an
%   optional prefix for the output filenames.

if nargin < 2 || isempty(output_dir)
    output_dir = 'results';
end
if nargin < 3
    tag = '';
end

if ~exist(output_dir, 'dir'); mkdir(output_dir); end
prefix = '';
if ~isempty(tag); prefix = [tag '_']; end

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
end

mean_pos = mean(res_pos, 1);
std_pos = std(res_pos, [], 1);
mean_vel = mean(res_vel, 1);
std_vel = std(res_vel, [], 1);
fprintf('Position residual mean [m]: %s\n', mat2str(mean_pos,3));
fprintf('Position residual std  [m]: %s\n', mat2str(std_pos,3));
fprintf('Velocity residual mean [m/s]: %s\n', mat2str(mean_vel,3));
fprintf('Velocity residual std  [m/s]: %s\n', mat2str(std_vel,3));

labels = {'X','Y','Z'};
f = figure('Visible','off','Position',[100 100 900 450]);
for i = 1:3
    subplot(2,3,i); plot(t, res_pos(:,i)); title(labels{i}); ylabel('Pos Residual [m]'); grid on;
    subplot(2,3,i+3); plot(t, res_vel(:,i)); xlabel('Time [s]'); ylabel('Vel Residual [m/s]'); grid on;
end
sgtitle('Task 7 - GNSS - Predicted Residuals');
set(f,'PaperPositionMode','auto');
pdf = fullfile(output_dir, sprintf('%stask7_residuals_position_velocity.pdf', prefix));
print(f, pdf, '-dpdf');
close(f); fprintf('Saved %s\n', pdf);

eul = rad2deg(quat2eul(quat(:,[2 3 4 1])));
f = figure('Visible','off','Position',[100 100 600 500]);
names = {'Roll','Pitch','Yaw'};
for i = 1:3
    subplot(3,1,i); plot(t, eul(:,i)); ylabel([names{i} ' [deg]']); grid on;
end
xlabel('Time [s]'); sgtitle('Task 7 - Attitude Angles');
set(f,'PaperPositionMode','auto');
pdf_att = fullfile(output_dir, sprintf('%stask7_attitude_angles_euler.pdf', prefix));
print(f, pdf_att, '-dpdf'); close(f); fprintf('Saved %s\n', pdf_att);

norm_pos = vecnorm(res_pos,2,2);
norm_vel = vecnorm(res_vel,2,2);
res_acc = gradient(res_vel, t);
norm_acc = vecnorm(res_acc,2,2);

f = figure('Visible','off');
plot(t, norm_pos, 'DisplayName','|pos error|'); hold on;
plot(t, norm_vel, 'DisplayName','|vel error|');
plot(t, norm_acc, 'DisplayName','|acc error|');
xlabel('Time [s]'); ylabel('Error Norm'); legend; grid on;
set(f,'PaperPositionMode','auto');
pdf_norm = fullfile(output_dir, sprintf('%stask7_error_norms.pdf', prefix));
print(f, pdf_norm, '-dpdf'); close(f); fprintf('Saved %s\n', pdf_norm);

rmse_pos = sqrt(mean(norm_pos.^2));
rmse_vel = sqrt(mean(norm_vel.^2));
rmse_acc = sqrt(mean(norm_acc.^2));
final_pos = norm_pos(end);
final_vel = norm_vel(end);
final_acc = norm_acc(end);

fprintf('[SUMMARY] rmse_pos=%.3f m final_pos=%.3f m rmse_vel=%.3f m/s final_vel=%.3f m/s\n', ...
    rmse_pos, final_pos, rmse_vel, final_vel);

end

