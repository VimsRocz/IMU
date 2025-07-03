function plot_overlay_with_truth(t_imu, pos_imu, vel_imu, acc_imu, ...
    t_gnss, pos_gnss, vel_gnss, acc_gnss, ...
    t_truth, pos_truth, vel_truth, acc_truth, ...
    frame, method, out_dir)
%PLOT_OVERLAY_WITH_TRUTH Overlay IMU/GNSS results with ground truth.
%   Saves a PDF similar to the Python overlay plots but including the
%   ground truth trajectory.

if nargin < 13
    error('Not enough input arguments.');
end

fig = figure('Visible','off', 'Position',[100 100 800 800]);

norm_imu   = vecnorm(pos_imu,2,2);
norm_gnss  = vecnorm(pos_gnss,2,2);
norm_truth = vecnorm(pos_truth,2,2);

subplot(4,1,1); hold on;
plot(t_imu,  norm_imu,  'b--', 'DisplayName','IMU');
plot(t_gnss, norm_gnss, 'k.', 'DisplayName','GNSS');
plot(t_truth,norm_truth,'g-', 'DisplayName','Truth');
ylabel('Position [m]'); grid on; legend;

a = vecnorm(vel_imu,2,2); b = vecnorm(vel_gnss,2,2); c = vecnorm(vel_truth,2,2);
subplot(4,1,2); hold on;
plot(t_imu,  a, 'b--');
plot(t_gnss, b, 'k.');
plot(t_truth,c, 'g-');
ylabel('Velocity [m/s]'); grid on;

a = vecnorm(acc_imu,2,2); b = vecnorm(acc_gnss,2,2); c = vecnorm(acc_truth,2,2);
subplot(4,1,3); hold on;
plot(t_imu,  a, 'b--');
plot(t_gnss, b, 'k.');
plot(t_truth,c, 'g-');
ylabel('Acceleration [m/s^2]'); grid on;

subplot(4,1,4); hold on;
plot(pos_imu(:,1), pos_imu(:,2), 'b--', 'DisplayName','IMU');
plot(pos_gnss(:,1), pos_gnss(:,2), 'k.', 'DisplayName','GNSS');
plot(pos_truth(:,1), pos_truth(:,2), 'g-', 'DisplayName','Truth');
xlabel([frame ' X']); ylabel([frame ' Y']); axis equal; grid on; legend;

sgtitle(sprintf('%s - %s frame overlay', method, frame));

if ~exist(out_dir,'dir'); mkdir(out_dir); end
fname = fullfile(out_dir, sprintf('%s_%s_truth_overlay.pdf', method, frame));
set(fig,'PaperPositionMode','auto');
print(fig, fname, '-dpdf','-bestfit');
close(fig);
end
