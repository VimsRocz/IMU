function fig = plot_task4_frame(t_imu, pos_imu, vel_imu, acc_imu, t_gnss, pos_gnss, vel_gnss, acc_gnss, varargin)
%PLOT_TASK4_FRAME  Plot Task 4 results in a 3x3 grid.
%   FIG = PLOT_TASK4_FRAME(T_IMU, POS_IMU, VEL_IMU, ACC_IMU, T_GNSS,
%   POS_GNSS, VEL_GNSS, ACC_GNSS) creates a figure with three rows
%   (position, velocity, acceleration) and three columns (axes N/E/D or
%   X/Y/Z). Optional name-value pairs:
%       'labels'     - cell array of axis labels {'N','E','D'}
%       'frame_name' - used for the figure title
%       't_fused'    - time vector for fused results
%       'pos_fused'  - fused position [Nx3]
%       'vel_fused'  - fused velocity [Nx3]
%       'acc_fused'  - fused acceleration [Nx3]
%       't_truth'    - time vector for ground truth
%       'pos_truth'  - ground truth position [Nx3]
%       'vel_truth'  - ground truth velocity [Nx3]
%       'acc_truth'  - ground truth acceleration [Nx3]
%       'method'     - method name appended to legends
%       'visible'    - figure visibility ('on' or 'off')
%
%   This function mirrors the plotting layout used in the Python
%   implementation.

p = inputParser;
addParameter(p, 'labels', {'N','E','D'});
addParameter(p, 'frame_name', 'NED');
addParameter(p, 't_fused', []);
addParameter(p, 'pos_fused', []);
addParameter(p, 'vel_fused', []);
addParameter(p, 'acc_fused', []);
addParameter(p, 't_truth', []);
addParameter(p, 'pos_truth', []);
addParameter(p, 'vel_truth', []);
addParameter(p, 'acc_truth', []);
addParameter(p, 'method', '');
addParameter(p, 'visible', 'on');
parse(p, varargin{:});
opt = p.Results;

fig = figure('Visible', opt.visible, 'Position',[100 100 1200 900]);

series_names = {'GNSS','IMU-derived','Fused','Truth'};

for j = 1:3
    %% Position
    ax = subplot(3,3,j); hold(ax,'on'); grid(ax,'on');
    if ~isempty(pos_gnss)
        plot(ax, t_gnss, pos_gnss(:,j), 'k-', 'DisplayName', series_names{1});
    end
    plot(ax, t_imu, pos_imu(:,j), 'b-', 'DisplayName', series_names{2});
    if ~isempty(opt.t_fused) && ~isempty(opt.pos_fused)
        plot(ax, opt.t_fused, opt.pos_fused(:,j), 'r-', 'DisplayName', series_names{3});
    end
    if ~isempty(opt.t_truth) && ~isempty(opt.pos_truth)
        plot(ax, opt.t_truth, opt.pos_truth(:,j), 'm-', 'DisplayName', series_names{4});
    end
    xlabel(ax, 'Time [s]');
    ylabel(ax, sprintf('Position %s [m]', opt.labels{j}));
    title(ax, sprintf('Position %s', opt.labels{j}));
    legend(ax,'Location','best');

    %% Velocity
    ax = subplot(3,3,3+j); hold(ax,'on'); grid(ax,'on');
    if ~isempty(vel_gnss)
        plot(ax, t_gnss, vel_gnss(:,j), 'k-', 'DisplayName', series_names{1});
    end
    plot(ax, t_imu, vel_imu(:,j), 'b-', 'DisplayName', series_names{2});
    if ~isempty(opt.t_fused) && ~isempty(opt.vel_fused)
        plot(ax, opt.t_fused, opt.vel_fused(:,j), 'r-', 'DisplayName', series_names{3});
    end
    if ~isempty(opt.t_truth) && ~isempty(opt.vel_truth)
        plot(ax, opt.t_truth, opt.vel_truth(:,j), 'm-', 'DisplayName', series_names{4});
    end
    xlabel(ax, 'Time [s]');
    ylabel(ax, sprintf('Velocity %s [m/s]', opt.labels{j}));
    title(ax, sprintf('Velocity %s', opt.labels{j}));
    legend(ax,'Location','best');

    %% Acceleration
    ax = subplot(3,3,6+j); hold(ax,'on'); grid(ax,'on');
    if ~isempty(acc_gnss)
        plot(ax, t_gnss, acc_gnss(:,j), 'k-', 'DisplayName', series_names{1});
    end
    plot(ax, t_imu, acc_imu(:,j), 'b-', 'DisplayName', series_names{2});
    if ~isempty(opt.t_fused) && ~isempty(opt.acc_fused)
        plot(ax, opt.t_fused, opt.acc_fused(:,j), 'r-', 'DisplayName', series_names{3});
    end
    if ~isempty(opt.t_truth) && ~isempty(opt.acc_truth)
        plot(ax, opt.t_truth, opt.acc_truth(:,j), 'm-', 'DisplayName', series_names{4});
    end
    xlabel(ax, 'Time [s]');
    ylabel(ax, sprintf('Acceleration %s [m/s^2]', opt.labels{j}));
    title(ax, sprintf('Acceleration %s', opt.labels{j}));
    legend(ax,'Location','best');
end

sgtitle(sprintf('Task 4: Trajectory Results in %s Frame', opt.frame_name));
end
