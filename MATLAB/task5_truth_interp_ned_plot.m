function task5_truth_interp_ned_plot(truth_time, truth_pos_ned, truth_vel_ned, imu_time, pos_interp, vel_interp, run_id, results_dir, cfg)
%TASK5_Truth_INTERP_NED_PLOT  Compare raw and interpolated Truth trajectories.
%   TASK5_Truth_INTERP_NED_PLOT(Truth_TIME, Truth_POS_NED, Truth_VEL_NED,
%   IMU_TIME, POS_INTERP, VEL_INTERP, RUN_ID, RESULTS_DIR, CFG) creates
%   position and velocity plots in the NED frame showing raw Truth samples
%   and the interpolated trajectories at IMU timestamps.  Plots are saved in
%   the provided results directory with filenames based on RUN_ID. CFG
%   controls plot visibility and saving policy.

if nargin < 8 || isempty(results_dir)
    results_dir = 'results';
end
if nargin < 9 || isempty(cfg)
    cfg = default_cfg();
end
visibleFlag = 'off';
try
    if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
        visibleFlag = 'on';
    end
catch
end

comp_labels = { 'North (m)', 'East (m)', 'Down (m)' };

% Position comparison ---------------------------------------------------
fig = figure('Name', 'Truth Position Interpolation', 'Position', [100 100 1200 600], ...
    'Visible', visibleFlag);
for i = 1:3
    subplot(2,3,i); hold on; grid on; box on;
    plot(truth_time, truth_pos_ned(:,i), 'o', 'DisplayName', 'Truth raw');
    plot(imu_time, pos_interp(:,i), '-', 'DisplayName', 'Interpolated');
    xlabel('Time (s)'); ylabel(comp_labels{i});
    title(sprintf('Position %s', comp_labels{i}));
    legend('Location','best');
end

% Velocity comparison ---------------------------------------------------
for i = 1:3
    subplot(2,3,i+3); hold on; grid on; box on;
    plot(truth_time, truth_vel_ned(:,i), 'o', 'DisplayName', 'Truth raw');
    plot(imu_time, vel_interp(:,i), '-', 'DisplayName', 'Interpolated');
    xlabel('Time (s)'); ylabel([comp_labels{i}(1:end-4) ' velocity (m/s)']);
    title(sprintf('Velocity %s', comp_labels{i}));
    legend('Location','best');
end

fname = fullfile(results_dir, sprintf('%s_task5_truth_interp_ned', run_id));
% Use best-fit printing to avoid page cut-off warnings
set(fig, 'PaperPositionMode', 'auto');
if cfg.plots.save_pdf
    print(fig, [fname '.pdf'], '-dpdf', '-bestfit');
end
if cfg.plots.save_png
    try
        exportgraphics(fig, [fname '.png'], 'Resolution', 300);
    catch
        % Fallback for older MATLAB versions
        print(fig, [fname '.png'], '-dpng', '-r300');
    end
end
close(fig);
end
