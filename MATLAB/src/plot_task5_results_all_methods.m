function plot_task5_results_all_methods(imu_time, fused_pos, fused_vel, fused_acc, ...
    gnss_pos_ned_interp, gnss_vel_ned_interp, gnss_acc_ned_interp, tag, results_dir)
%PLOT_TASK5_RESULTS_ALL_METHODS  Plot fused results from all methods versus GNSS.
%   PLOT_TASK5_RESULTS_ALL_METHODS(IMU_TIME, FUSED_POS, FUSED_VEL, FUSED_ACC,
%   GNSS_POS_NED_INTERP, GNSS_VEL_NED_INTERP, GNSS_ACC_NED_INTERP) creates a
%   3x3 subplot figure comparing the fused position, velocity and acceleration
%   of the TRIAD, Davenport and SVD methods against the interpolated GNSS
%   measurements.  Each row corresponds to position, velocity and acceleration
%   while the columns represent the North, East and Down axes.
%   The figure is saved as 'task5_results_all_methods.png' in the current
%   directory.
%
%   The function also prints the first and last value of each plotted series
%   (GNSS and all methods) for every axis, mimicking the Python logging used in
%   the reference implementation.
%
%   Table of figures for Task 5 plots:
%   | Figure File                   | Content                                                        |
%   | ----------------------------- | -------------------------------------------------------------- |
%   | task5_results_TRIAD.png       | 3x3: GNSS vs Fused TRIAD—Position, Velocity, Acceleration      |
%   | task5_results_Davenport.png   | 3x3: GNSS vs Fused Davenport—Position, Velocity, Acceleration |
%   | task5_results_SVD.png         | 3x3: GNSS vs Fused SVD—Position, Velocity, Acceleration       |
%   | task5_results_all_methods.png | 3x3: GNSS, TRIAD, Davenport, SVD—Position, Velocity, Accel.   |
%
%   Example usage:
%       plot_task5_results_all_methods(t, posStruct, velStruct, accStruct,
%           gnssPos, gnssVel, gnssAcc);
%
%   where POSSTRUCT, VELSTRUCT and ACCSTRUCT have fields 'TRIAD', 'Davenport'
%   and 'SVD', each containing an Nx3 matrix.

labels = {'North','East','Down'};
methods = {'TRIAD','Davenport','SVD'};
method_colors = {'r','g','b'};

if nargin < 8 || isempty(results_dir)
    results_dir = get_results_dir();
end
if ~exist(results_dir,'dir'); mkdir(results_dir); end
fig = figure('Name','Task5 Results - All Methods', 'Position',[100 100 1200 900]);

for j = 1:3
    %% Position subplot
    ax = subplot(3,3,j); hold(ax,'on'); grid(ax,'on');
    plot(imu_time, gnss_pos_ned_interp(:,j), 'k-', 'LineWidth',2, 'DisplayName','GNSS');
    for m = 1:numel(methods)
        plot(imu_time, fused_pos.(methods{m})(:,j), method_colors{m}, 'LineWidth',2, ...
            'DisplayName', methods{m});
    end
    xlabel(ax,'Time [s]');
    ylabel(ax,['Position ' labels{j} ' [m]']);
    title(ax,['Position ' labels{j}]);
    legend(ax,'Location','best');
    set(ax,'FontSize',14);

    fprintf('# GNSS position %s: First = %.4f, Last = %.4f\n', ...
        labels{j}, gnss_pos_ned_interp(1,j), gnss_pos_ned_interp(end,j));
    for m = 1:numel(methods)
        fp = fused_pos.(methods{m});
        fprintf('# %s position %s: First = %.4f, Last = %.4f\n', ...
            methods{m}, labels{j}, fp(1,j), fp(end,j));
    end

    %% Velocity subplot
    ax = subplot(3,3,3+j); hold(ax,'on'); grid(ax,'on');
    plot(imu_time, gnss_vel_ned_interp(:,j), 'k-', 'LineWidth',2, 'DisplayName','GNSS');
    for m = 1:numel(methods)
        plot(imu_time, fused_vel.(methods{m})(:,j), method_colors{m}, 'LineWidth',2, ...
            'DisplayName', methods{m});
    end
    xlabel(ax,'Time [s]');
    ylabel(ax,['Velocity ' labels{j} ' [m/s]']);
    title(ax,['Velocity ' labels{j}]);
    legend(ax,'Location','best');
    set(ax,'FontSize',14);

    fprintf('# GNSS velocity %s: First = %.4f, Last = %.4f\n', ...
        labels{j}, gnss_vel_ned_interp(1,j), gnss_vel_ned_interp(end,j));
    for m = 1:numel(methods)
        fv = fused_vel.(methods{m});
        fprintf('# %s velocity %s: First = %.4f, Last = %.4f\n', ...
            methods{m}, labels{j}, fv(1,j), fv(end,j));
    end

    %% Acceleration subplot
    ax = subplot(3,3,6+j); hold(ax,'on'); grid(ax,'on');
    plot(imu_time, gnss_acc_ned_interp(:,j), 'k-', 'LineWidth',2, 'DisplayName','GNSS');
    for m = 1:numel(methods)
        plot(imu_time, fused_acc.(methods{m})(:,j), method_colors{m}, 'LineWidth',2, ...
            'DisplayName', methods{m});
    end
    xlabel(ax,'Time [s]');
    ylabel(ax,['Acceleration ' labels{j} ' [m/s^2]']);
    title(ax,['Acceleration ' labels{j}]);
    legend(ax,'Location','best');
    set(ax,'FontSize',14);

    fprintf('# GNSS acceleration %s: First = %.4f, Last = %.4f\n', ...
        labels{j}, gnss_acc_ned_interp(1,j), gnss_acc_ned_interp(end,j));
    for m = 1:numel(methods)
        fa = fused_acc.(methods{m});
        fprintf('# %s acceleration %s: First = %.4f, Last = %.4f\n', ...
            methods{m}, labels{j}, fa(1,j), fa(end,j));
    end
end

sgtitle('Task 5 Comparison - All Methods','FontSize',14);
set(fig,'PaperPositionMode','auto');
pdf_path = fullfile(results_dir, sprintf('%s_task5_results_all_methods.pdf', tag));
print(fig, pdf_path, '-dpdf', '-bestfit');
fprintf('Saved plot to %s\n', pdf_path);
close(fig);
close(fig);
end
