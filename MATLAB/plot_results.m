function plot_results(matFile)
% Load .mat result and recreate plots for attitude, residuals and NED
% comparison.  Figures are saved next to the input file using the same
% naming scheme as the Python scripts.

S = load(matFile);
[~,name] = fileparts(matFile);

if isfield(S, 'euler')
    figure; plot(S.euler);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    legend('roll','pitch','yaw');
    title('Attitude Angles');
    saveas(gcf, [name '_attitude.pdf']);
end

if isfield(S, 'residual_pos')
    figure; plot(S.residual_pos);
    xlabel('Time [s]'); ylabel('Position Residual [m]');
    legend('N','E','D');
    title('Position Residuals');
    saveas(gcf, [name '_pos_residuals.pdf']);
end

if isfield(S, 'residual_vel')
    figure; plot(S.residual_vel);
    xlabel('Time [s]'); ylabel('Velocity Residual [m/s]');
    legend('N','E','D');
    title('Velocity Residuals');
    saveas(gcf, [name '_vel_residuals.pdf']);
end
end
