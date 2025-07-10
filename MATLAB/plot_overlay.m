function plot_overlay(frame, method, t_imu, pos_imu, vel_imu, acc_imu, ...
    t_gnss, pos_gnss, vel_gnss, acc_gnss, t_fused, pos_fused, vel_fused, acc_fused, out_dir, varargin)
%PLOT_OVERLAY  Save overlay plot comparing measured IMU, GNSS and fused tracks.
%   PLOT_OVERLAY(FRAME, METHOD, T_IMU, POS_IMU, VEL_IMU, ACC_IMU, T_GNSS,
%   POS_GNSS, VEL_GNSS, ACC_GNSS, T_FUSED, POS_FUSED, VEL_FUSED, ACC_FUSED,
%   OUT_DIR) creates a 4x1 subplot figure showing the norms of position,
%   velocity and acceleration as well as the XY trajectory. The figure is
%   saved as METHOD_FRAME_overlay.pdf in OUT_DIR.
%
%   Optional name-value pairs:
%     't_truth'   - time vector for ground truth
%     'pos_truth' - ground truth positions
%     'vel_truth' - ground truth velocities
%     'acc_truth' - ground truth accelerations
%     'suffix'    - custom file suffix
%
%   When ground truth data is provided the default suffix becomes
%   '_overlay_truth.pdf'.

p = inputParser;
addParameter(p, 't_truth', []);
addParameter(p, 'pos_truth', []);
addParameter(p, 'vel_truth', []);
addParameter(p, 'acc_truth', []);
addParameter(p, 'suffix', '');
parse(p, varargin{:});
Ttruth = p.Results.t_truth;
ptruth = p.Results.pos_truth;
vtruth = p.Results.vel_truth;
atruth = p.Results.acc_truth;
suffix = p.Results.suffix;

if isempty(suffix)
    if ~isempty(Ttruth)
        suffix = '_overlay_truth.pdf';
    else
        suffix = '_overlay.pdf';
    end
end

h = figure('Visible','off');

subplot(4,1,1); hold on;
plot(t_gnss, vecnorm(pos_gnss,2,2), 'k-', 'DisplayName', 'Measured GNSS');
plot(t_imu, vecnorm(pos_imu,2,2), 'c--', 'DisplayName', 'Derived IMU');
if ~isempty(Ttruth) && ~isempty(ptruth)
    plot(Ttruth, vecnorm(ptruth,2,2), 'm-', 'DisplayName', 'Truth');
end
plot(t_fused, vecnorm(pos_fused,2,2), 'g:', 'DisplayName', ['Fused ' method]);
ylabel('Position [m]');
legend('show');

subplot(4,1,2); hold on;
plot(t_gnss, vecnorm(vel_gnss,2,2), 'k-', 'DisplayName', 'Measured GNSS');
plot(t_imu, vecnorm(vel_imu,2,2), 'c--', 'DisplayName', 'Derived IMU');
if ~isempty(Ttruth) && ~isempty(vtruth)
    plot(Ttruth, vecnorm(vtruth,2,2), 'm-', 'DisplayName', 'Truth');
end
plot(t_fused, vecnorm(vel_fused,2,2), 'g:', 'DisplayName', ['Fused ' method]);
ylabel('Velocity [m/s]');

subplot(4,1,3); hold on;
plot(t_gnss, vecnorm(acc_gnss,2,2), 'k-', 'DisplayName', 'Measured GNSS');
plot(t_imu, vecnorm(acc_imu,2,2), 'c--', 'DisplayName', 'Derived IMU');
if ~isempty(Ttruth) && ~isempty(atruth)
    plot(Ttruth, vecnorm(atruth,2,2), 'm-', 'DisplayName', 'Truth');
end
plot(t_fused, vecnorm(acc_fused,2,2), 'g:', 'DisplayName', ['Fused ' method]);
ylabel('Acceleration [m/s^2]');

subplot(4,1,4); hold on;
plot(pos_gnss(:,1), pos_gnss(:,2), 'k-', 'DisplayName', 'Measured GNSS');
plot(pos_imu(:,1), pos_imu(:,2), 'c--', 'DisplayName', 'Derived IMU');
if ~isempty(ptruth)
    plot(ptruth(:,1), ptruth(:,2), 'm-', 'DisplayName', 'Truth');
end
plot(pos_fused(:,1), pos_fused(:,2), 'g:', 'DisplayName', ['Fused ' method]);
xlabel([frame ' X']);
ylabel([frame ' Y']);
title('Trajectory');
axis equal;

sgtitle([method ' - ' frame ' frame comparison']);
set(h,'PaperPositionMode','auto');
out_file = fullfile(out_dir, [method '_' frame suffix]);
print(h, out_file, '-dpdf', '-bestfit');
close(h);
end
