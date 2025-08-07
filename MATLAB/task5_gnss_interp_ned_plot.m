function task5_gnss_interp_ned_plot(gnss_time, gnss_pos_ned, gnss_vel_ned, imu_time, pos_interp, vel_interp, tag, results_dir)
%TASK5_GNSS_INTERP_NED_PLOT  Compare raw and interpolated GNSS trajectories.
%   TASK5_GNSS_INTERP_NED_PLOT(GNSS_TIME, GNSS_POS_NED, GNSS_VEL_NED,
%   IMU_TIME, POS_INTERP, VEL_INTERP, TAG, RESULTS_DIR) creates position and
%   velocity plots in the NED frame showing raw GNSS samples and the
%   interpolated trajectories at IMU timestamps.  Plots are saved in the
%   provided results directory with filenames based on TAG.
%
%   Usage:
%       task5_gnss_interp_ned_plot(t_gnss, pos_gnss, vel_gnss, t_imu,
%                                  pos_interp, vel_interp, tag, out_dir)

if nargin < 8
    results_dir = get_results_dir();
end

comp_labels = { 'North (m)', 'East (m)', 'Down (m)' };

% Position comparison ---------------------------------------------------
fig = figure('Name', 'GNSS Position Interpolation', 'Position', [100 100 1200 600]);
for i = 1:3
    subplot(2,3,i); hold on; grid on; box on;
    plot(gnss_time, gnss_pos_ned(:,i), 'o', 'DisplayName', 'GNSS raw');
    plot(imu_time, pos_interp(:,i), '-', 'DisplayName', 'Interpolated');
    xlabel('Time (s)'); ylabel(comp_labels{i});
    title(sprintf('Position %s', comp_labels{i}));
    legend('Location','best');
end

% Velocity comparison ---------------------------------------------------
for i = 1:3
    subplot(2,3,i+3); hold on; grid on; box on;
    plot(gnss_time, gnss_vel_ned(:,i), 'o', 'DisplayName', 'GNSS raw');
    plot(imu_time, vel_interp(:,i), '-', 'DisplayName', 'Interpolated');
    xlabel('Time (s)'); ylabel([comp_labels{i}(1:end-4) ' velocity (m/s)']);
    title(sprintf('Velocity %s', comp_labels{i}));
    legend('Location','best');
end

fname = fullfile(results_dir, sprintf('%s_task5_gnss_interp_ned', tag));
saveas(fig, [fname '.png']);
saveas(fig, [fname '.pdf']);
close(fig);
end
