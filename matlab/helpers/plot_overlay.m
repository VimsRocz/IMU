function plot_overlay(t_imu, pos_imu, vel_imu, acc_imu, t_gnss, pos_gnss, vel_gnss, acc_gnss, t_fused, pos_fused, vel_fused, acc_fused, frame, method, outdir)
%PLOT_OVERLAY Create 4x1 overlay plot for IMU-only, GNSS and fused data.
fig = figure('Visible','off');
subplot(4,1,1); hold on;
plot(t_imu, vecnorm(pos_imu,2,2),'b--');
plot(t_gnss, vecnorm(pos_gnss,2,2),'k.');
plot(t_fused, vecnorm(pos_fused,2,2),'r-');
ylabel('Pos [m]'); legend('IMU','GNSS','Fused');
subplot(4,1,2); hold on;
plot(t_imu, vecnorm(vel_imu,2,2),'b--');
plot(t_gnss, vecnorm(vel_gnss,2,2),'k.');
plot(t_fused, vecnorm(vel_fused,2,2),'r-');
ylabel('Vel [m/s]');
subplot(4,1,3); hold on;
plot(t_imu, vecnorm(acc_imu,2,2),'b--');
plot(t_gnss, vecnorm(acc_gnss,2,2),'k.');
plot(t_fused, vecnorm(acc_fused,2,2),'r-');
ylabel('Acc [m/s^2]');
subplot(4,1,4); hold on;
plot(pos_imu(:,1), pos_imu(:,2),'b--');
plot(pos_gnss(:,1), pos_gnss(:,2),'k.');
plot(pos_fused(:,1), pos_fused(:,2),'r-');
xlabel([frame ' X']); ylabel([frame ' Y']); title('Trajectory'); axis equal;
sgtitle([method ' ' frame ' frame comparison']);
filename = fullfile(outdir, sprintf('%s_%s_overlay.pdf', method, frame));
print(fig, filename, '-dpdf');
close(fig);
end
