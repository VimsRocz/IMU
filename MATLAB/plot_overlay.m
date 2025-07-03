function plot_overlay(frame, method, t_imu, pos_imu, vel_imu, acc_imu, ...
    t_gnss, pos_gnss, vel_gnss, acc_gnss, t_fused, pos_fused, vel_fused, ...
    acc_fused, out_dir, varargin)
%PLOT_OVERLAY  Overlay IMU, GNSS and fused tracks in a single figure.
%   PLOT_OVERLAY(FRAME, METHOD, T_IMU, POS_IMU, VEL_IMU, ACC_IMU,
%   T_GNSS, POS_GNSS, VEL_GNSS, ACC_GNSS, T_FUSED, POS_FUSED, VEL_FUSED,
%   ACC_FUSED, OUT_DIR) saves a 4x1 panel comparing the IMU-only,
%   GNSS and fused solutions. Optional name-value pairs:
%       't_truth'   - time vector for ground truth
%       'pos_truth' - position samples for ground truth
%       'vel_truth' - velocity samples for ground truth
%       'acc_truth' - acceleration samples for ground truth
%       'suffix'    - custom file suffix (defaults to _overlay.pdf or
%                     _overlay_truth.pdf when truth arrays are present)
%
%   The resulting PDF is written as <METHOD>_<FRAME><SUFFIX> inside
%   OUT_DIR. FRAME is used only for axis labels and the title.

p = inputParser;
addParameter(p, 't_truth', []);
addParameter(p, 'pos_truth', []);
addParameter(p, 'vel_truth', []);
addParameter(p, 'acc_truth', []);
addParameter(p, 'suffix', '');
parse(p, varargin{:});
Ttruth = p.Results.t_truth;
Postruth = p.Results.pos_truth;
Veltruth = p.Results.vel_truth;
Acctruth = p.Results.acc_truth;
suffix = p.Results.suffix;

if isempty(suffix)
    if ~isempty(Ttruth)
        suffix = '_overlay_truth.pdf';
    else
        suffix = '_overlay.pdf';
    end
end

fig = figure('Visible','off','Position',[100 100 600 800]);

subplot(4,1,1);
hold on;
plot(t_imu, vecnorm(pos_imu,2,2),'b--','DisplayName','IMU only');
plot(t_gnss, vecnorm(pos_gnss,2,2),'k.','DisplayName','GNSS');
if ~isempty(Ttruth) && ~isempty(Postruth)
    plot(Ttruth, vecnorm(Postruth,2,2),'g-','DisplayName','Truth');
end
plot(t_fused, vecnorm(pos_fused,2,2),'r-','DisplayName','Fused');
ylabel('Position [m]');
legend;

subplot(4,1,2);
hold on;
plot(t_imu, vecnorm(vel_imu,2,2),'b--');
plot(t_gnss, vecnorm(vel_gnss,2,2),'k.');
if ~isempty(Ttruth) && ~isempty(Veltruth)
    plot(Ttruth, vecnorm(Veltruth,2,2),'g-');
end
plot(t_fused, vecnorm(vel_fused,2,2),'r-');
ylabel('Velocity [m/s]');

subplot(4,1,3);
hold on;
plot(t_imu, vecnorm(acc_imu,2,2),'b--');
plot(t_gnss, vecnorm(acc_gnss,2,2),'k.');
if ~isempty(Ttruth) && ~isempty(Acctruth)
    plot(Ttruth, vecnorm(Acctruth,2,2),'g-');
end
plot(t_fused, vecnorm(acc_fused,2,2),'r-');
ylabel('Acceleration [m/s^2]');

subplot(4,1,4);
hold on;
plot(pos_imu(:,1), pos_imu(:,2),'b--','DisplayName','IMU only');
plot(pos_gnss(:,1), pos_gnss(:,2),'k.','DisplayName','GNSS');
if ~isempty(Postruth)
    plot(Postruth(:,1), Postruth(:,2),'g-','DisplayName','Truth');
end
plot(pos_fused(:,1), pos_fused(:,2),'r-','DisplayName','Fused');
xlabel([frame ' X']);
ylabel([frame ' Y']);
title('Trajectory');
axis equal;

sgtitle(sprintf('%s - %s frame comparison', method, frame));

out_path = fullfile(out_dir, sprintf('%s_%s%s', method, frame, suffix));
print(fig, out_path, '-dpdf');
close(fig);
end
