function varargout = Task_5(varargin)
% Unified Task 5 entry point. Delegates to original implementation.
if nargout
    [varargout{1:nargout}] = MATLAB.Task_5.Task_5(varargin{:});
else
    MATLAB.Task_5.Task_5(varargin{:});
end
end

% Local helper consolidated from MATLAB/Task_5/task5_gnss_interp_ned_plot.m
function task5_gnss_interp_ned_plot(gnss_time, gnss_pos_ned, gnss_vel_ned, imu_time, pos_interp, vel_interp, run_id, results_dir, cfg)
%TASK5_GNSS_INTERP_NED_PLOT  Compare raw and interpolated GNSS trajectories.
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
fig = figure('Name', 'GNSS Position Interpolation', 'Position', [100 100 1200 600], 'Visible', visibleFlag);
for i = 1:3
    subplot(2,3,i); hold on; grid on; box on;
    plot(gnss_time, gnss_pos_ned(:,i), 'o', 'DisplayName', 'GNSS raw');
    plot(imu_time, pos_interp(:,i), '-', 'DisplayName', 'Interpolated');
    xlabel('Time (s)'); ylabel(comp_labels{i});
    title(sprintf('Position %s', comp_labels{i}));
    legend('Location','best');
end
for i = 1:3
    subplot(2,3,i+3); hold on; grid on; box on;
    plot(gnss_time, gnss_vel_ned(:,i), 'o', 'DisplayName', 'GNSS raw');
    plot(imu_time, vel_interp(:,i), '-', 'DisplayName', 'Interpolated');
    xlabel('Time (s)'); ylabel([comp_labels{i}(1:end-4) ' velocity (m/s)']);
    title(sprintf('Velocity %s', comp_labels{i}));
    legend('Location','best');
end
fname = fullfile(results_dir, sprintf('%s_task5_gnss_interp_ned', run_id));
set(fig, 'PaperPositionMode', 'auto');
if isfield(cfg,'plots') && isfield(cfg.plots,'save_png') && cfg.plots.save_png
    try
        exportgraphics(fig, [fname '.png'], 'Resolution', 300);
    catch
        % Fallback for older MATLAB versions
        try, savefig(fig, [fname '.fig']); catch, end
    end
end
try, savefig(fig, [fname '.fig']); catch, end
close(fig);
end
