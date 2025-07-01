function plot_results(resultFile)
%PLOT_RESULTS Recreate plots from saved result files.
%   PLOT_RESULTS(FILE) loads the given MAT or NPZ result file and produces
%   the attitude, position and velocity residual plots.  Figures are saved
%   next to the input file using the same naming scheme as the Python
%   scripts.

    [~,name,ext] = fileparts(resultFile);
    if strcmpi(ext, '.npz')
        if exist('load_npz', 'file')
            S = load_npz(resultFile);
        else
            error('NPZ file given but load_npz.m is missing.');
        end
    else
        S = load(resultFile);
    end

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
