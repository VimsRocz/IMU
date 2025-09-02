function task7_ecef_residuals_plot(est_file, imu_file, gnss_file, truth_file, dataset, output_dir)
%TASK7_ECEF_RESIDUALS_PLOT  Plot ECEF residuals for Task 7.
%
%   task7_ecef_residuals_plot(est_file, imu_file, gnss_file, truth_file, dataset, output_dir)
%   mirrors the Python script ``task7_ecef_residuals_plot.py``. The IMU and GNSS
%   file paths are currently unused but included for interface parity. The truth
%   data is interpolated to the estimator timestamps and residuals in position,
%   velocity and acceleration are plotted. The time axis is shifted so the
%   first sample corresponds to ``t=0`` matching Task 6. Figures are saved under
%   ``output_dir`` using ``dataset`` as part of the filename.

if nargin < 6 || isempty(output_dir)

end
out_dir = output_dir;
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

if (nargin < 4 || isempty(truth_file)) && nargin >= 5
    m = regexp(dataset, 'X(\d+)', 'tokens', 'once');
    if ~isempty(m)
        cand1 = fullfile(sprintf('STATE_X%s.txt', m{1}));
        cand2 = fullfile(sprintf('STATE_X%s_small.txt', m{1}));
        if exist(cand1, 'file')
            truth_file = cand1;
        elseif exist(cand2, 'file')
            truth_file = cand2;
        else
            error('Truth file not specified and could not be inferred from dataset');
        end
    end
end

[t_est, pos_est, vel_est, ~] = load_est(est_file);
[t_truth, pos_truth, vel_truth, ~] = load_est(truth_file);

% Use relative time to match Task 6 plots
t_rel = t_est - t_est(1);

pos_truth_i = interp1(t_truth, pos_truth, t_est, 'linear', 'extrap');
vel_truth_i = interp1(t_truth, vel_truth, t_est, 'linear', 'extrap');

[res_pos, res_vel, res_acc] = compute_residuals(t_rel, pos_est, vel_est, pos_truth_i, vel_truth_i);

plot_residuals(t_rel, res_pos, res_vel, res_acc, dataset, out_dir);

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
function plot_residuals(t, res_pos, res_vel, ~, dataset, out_dir)
%PLOT_RESIDUALS Plot residual components and norms (2x3 pos/vel vs X/Y/Z).

labels = {'X','Y','Z'};
f = figure('Visible','off');
set(f,'Units','centimeters','Position',[2 2 18 9]); % FIX: page width
set(f,'PaperPositionMode','auto');
for j = 1:3
    subplot(2,3,j); hold on; plot(t, res_pos(:,j)); grid on;
    title(labels{j}); ylabel('Position Residual [m]');
    if j==2, xlabel('Time [s]'); end
end
for j = 1:3
    subplot(2,3,3+j); hold on; plot(t, res_vel(:,j)); grid on;
    ylabel('Velocity Residual [m/s]'); xlabel('Time [s]');
end
sgtitle(sprintf('%s Task 7 ECEF Residuals (Pos/Vel)', dataset));
png = fullfile(out_dir, sprintf('%s_task7_ecef_residuals_pos_vel.png', dataset));
exportgraphics(f, png, 'Resolution',300);
close(f);

f = figure('Visible','off');
set(f,'Units','centimeters','Position',[2 2 18 9]);
set(f,'PaperPositionMode','auto');
plot(t, vecnorm(res_pos,2,2), 'DisplayName','|pos|'); hold on;
plot(t, vecnorm(res_vel,2,2), 'DisplayName','|vel|');
plot(t, vecnorm(res_acc,2,2), 'DisplayName','|acc|');
xlabel('Time [s]'); ylabel('Residual Norm'); legend; grid on;
pngn = fullfile(out_dir, sprintf('%s_task7_ecef_residual_norms.png', dataset));
exportgraphics(f, pngn, 'Resolution',300);
close(f);
files = dir(fullfile(out_dir, sprintf('%s_task7_ecef_residual*.pdf', dataset)));
if ~isempty(files)
    fprintf('Files saved in %s\n', out_dir);
    for k=1:numel(files)
        fprintf(' - %s\n', files(k).name);
    end
end
end
