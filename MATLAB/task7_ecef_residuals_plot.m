function task7_ecef_residuals_plot(est_file, imu_file, gnss_file, truth_file, dataset, output_dir)
%TASK7_ECEF_RESIDUALS_PLOT  Plot ECEF residuals for Task 7.
%
%   task7_ecef_residuals_plot(est_file, imu_file, gnss_file, truth_file, dataset, output_dir)
%   mirrors the Python script ``task7_ecef_residuals_plot.py``. The IMU and GNSS
%   file paths are currently unused but included for interface parity. The truth
%   data is interpolated to the estimator timestamps and residuals in position,
%   velocity and acceleration are plotted. Figures are saved under
%   ``output_dir`` using ``dataset`` as part of the filename.

if nargin < 6 || isempty(output_dir)
    output_dir = 'output_matlab';
end
out_dir = fullfile(output_dir, 'task7', dataset);
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

[t_est, pos_est, vel_est, ~] = load_est(est_file);
[t_truth, pos_truth, vel_truth, ~] = load_est(truth_file);

pos_truth_i = interp1(t_truth, pos_truth, t_est, 'linear', 'extrap');
vel_truth_i = interp1(t_truth, vel_truth, t_est, 'linear', 'extrap');

[res_pos, res_vel, res_acc] = compute_residuals(t_est, pos_est, vel_est, pos_truth_i, vel_truth_i);

plot_residuals(t_est, res_pos, res_vel, res_acc, dataset, out_dir);

end

% -------------------------------------------------------------------------
function [t, pos, vel, acc] = load_est(file)
%LOAD_EST Load NPZ/MAT estimate containing ECEF position and velocity.

fname = string(file);
if endsWith(fname,'.npz')
    d = py.numpy.load(fname);
    t = double(d{'time_s'});
    pos = double(d{'pos_ecef_m'});
    vel = double(d{'vel_ecef_ms'});
    if isKey(d,'acc_ecef_ms2')
        acc = double(d{'acc_ecef_ms2'});
    else
        acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
    end
else
    S = load(fname);
    t = S.time_s(:);
    pos = S.pos_ecef_m;
    vel = S.vel_ecef_ms;
    if isfield(S,'acc_ecef_ms2')
        acc = S.acc_ecef_ms2;
    else
        acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
    end
end
end

% -------------------------------------------------------------------------
function [res_pos, res_vel, res_acc] = compute_residuals(t, est_pos, est_vel, truth_pos, truth_vel)
%COMPUTE_RESIDUALS Return position, velocity and acceleration residuals.

res_pos = est_pos - truth_pos;
res_vel = est_vel - truth_vel;
acc_est = gradient(est_vel, t);
acc_truth = gradient(truth_vel, t);
res_acc = acc_est - acc_truth;
end

% -------------------------------------------------------------------------
function plot_residuals(t, res_pos, res_vel, res_acc, dataset, out_dir)
%PLOT_RESIDUALS Plot residual components and norms.

labels = {'X','Y','Z'};
f = figure('Visible','off','Position',[100 100 900 700]);
for i = 1:3
    for j = 1:3
        ax = subplot(3,3,(i-1)*3+j); hold on;
        switch i
            case 1; arr = res_pos; ylab = 'Position Residual [m]';
            case 2; arr = res_vel; ylab = 'Velocity Residual [m/s]';
            otherwise; arr = res_acc; ylab = 'Acceleration Residual [m/s^2]';
        end
        plot(t, arr(:,j));
        if i==1
            title(labels{j});
        end
        if j==1
            ylabel(ylab);
        end
        if i==3
            xlabel('Time [s]');
        end
        grid on;
    end
end
sgtitle(sprintf('%s Task 7 ECEF Residuals', dataset));
set(f,'PaperPositionMode','auto');
pdf = fullfile(out_dir, sprintf('%s_task7_ecef_residuals.pdf', dataset));
png = fullfile(out_dir, sprintf('%s_task7_ecef_residuals.png', dataset));
print(f, pdf, '-dpdf');
print(f, png, '-dpng');
close(f);

f = figure('Visible','off');
plot(t, vecnorm(res_pos,2,2), 'DisplayName','|pos|'); hold on;
plot(t, vecnorm(res_vel,2,2), 'DisplayName','|vel|');
plot(t, vecnorm(res_acc,2,2), 'DisplayName','|acc|');
xlabel('Time [s]'); ylabel('Residual Norm'); legend; grid on;
set(f,'PaperPositionMode','auto');
pdfn = fullfile(out_dir, sprintf('%s_task7_ecef_residual_norms.pdf', dataset));
pngn = fullfile(out_dir, sprintf('%s_task7_ecef_residual_norms.png', dataset));
print(f, pdfn, '-dpdf');
print(f, pngn, '-dpng');
close(f);
end

