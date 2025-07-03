function plot_overlay(frame, method, t_imu, pos_imu, vel_imu, acc_imu, ...
                      t_gnss, pos_gnss, vel_gnss, acc_gnss, ...
                      t_fused, pos_fused, vel_fused, acc_fused, out_dir, varargin)
%PLOT_OVERLAY  Overlay IMU, GNSS and fused data in one figure.
%   PLOT_OVERLAY(FRAME, METHOD, T_IMU, POS_IMU, VEL_IMU, ACC_IMU,
%   T_GNSS, POS_GNSS, VEL_GNSS, ACC_GNSS, T_FUSED, POS_FUSED, VEL_FUSED,
%   ACC_FUSED, OUT_DIR) creates a 4x1 plot comparing the norms of the
%   position, velocity and acceleration vectors together with the
%   X/Y trajectory.  The figure is saved as
%   <METHOD>_<FRAME>_overlay.pdf inside OUT_DIR.
%
%   Optional name/value pairs:
%       't_truth'   - time vector for ground truth
%       'pos_truth' - N-by-3 truth positions
%       'vel_truth' - N-by-3 truth velocities
%       'acc_truth' - N-by-3 truth accelerations
%       'suffix'    - output filename suffix (default '_overlay.pdf')
%
%   Example:
%       plot_overlay('NED', 'TRIAD', t1, p1, v1, a1, t2, p2, v2, a2,
%                    tf, pf, vf, af, 'results');
%

opts = struct('t_truth', [], 'pos_truth', [], 'vel_truth', [], ...
              'acc_truth', [], 'suffix', '_overlay.pdf');
for k = 1:2:numel(varargin)
    key = varargin{k};
    if k+1 <= numel(varargin)
        opts.(lower(key)) = varargin{k+1};
    end
end

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

h = figure('Visible','off');

subplot(4,1,1); hold on;
plot(t_imu, vecnorm(pos_imu,2,2), 'b--', 'DisplayName','IMU only');
plot(t_gnss, vecnorm(pos_gnss,2,2), 'k.', 'DisplayName','GNSS');
if ~isempty(opts.t_truth) && ~isempty(opts.pos_truth)
    plot(opts.t_truth, vecnorm(opts.pos_truth,2,2), 'k-', 'DisplayName','Truth');
end
plot(t_fused, vecnorm(pos_fused,2,2), 'r-', 'DisplayName','Fused');
ylabel('Position [m]');
legend('show');

subplot(4,1,2); hold on;
plot(t_imu, vecnorm(vel_imu,2,2), 'b--');
plot(t_gnss, vecnorm(vel_gnss,2,2), 'k.');
if ~isempty(opts.t_truth) && ~isempty(opts.vel_truth)
    plot(opts.t_truth, vecnorm(opts.vel_truth,2,2), 'k-');
end
plot(t_fused, vecnorm(vel_fused,2,2), 'r-');
ylabel('Velocity [m/s]');

subplot(4,1,3); hold on;
plot(t_imu, vecnorm(acc_imu,2,2), 'b--');
plot(t_gnss, vecnorm(acc_gnss,2,2), 'k.');
if ~isempty(opts.t_truth) && ~isempty(opts.acc_truth)
    plot(opts.t_truth, vecnorm(opts.acc_truth,2,2), 'k-');
end
plot(t_fused, vecnorm(acc_fused,2,2), 'r-');
ylabel('Acceleration [m/s^2]');

subplot(4,1,4); hold on;
plot(pos_imu(:,1), pos_imu(:,2), 'b--', 'DisplayName','IMU only');
plot(pos_gnss(:,1), pos_gnss(:,2), 'k.', 'DisplayName','GNSS');
if ~isempty(opts.pos_truth)
    plot(opts.pos_truth(:,1), opts.pos_truth(:,2), 'k-', 'DisplayName','Truth');
end
plot(pos_fused(:,1), pos_fused(:,2), 'r-', 'DisplayName','Fused');
xlabel([frame ' X']);
ylabel([frame ' Y']);
title('Trajectory');
axis equal;

sgtitle(sprintf('%s - %s frame comparison', method, frame));

out_path = fullfile(out_dir, sprintf('%s_%s%s', method, frame, opts.suffix));
print(h, out_path, '-dpdf');
close(h);
end
