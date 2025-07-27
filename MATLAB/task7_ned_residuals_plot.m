function task7_ned_residuals_plot(est_file, truth_file, dataset, output_dir)
%TASK7_NED_RESIDUALS_PLOT  Plot NED residuals for Task 7.
%   TASK7_NED_RESIDUALS_PLOT(EST_FILE, TRUTH_FILE, DATASET, OUTPUT_DIR) loads
%   a fused estimator result and the ground truth trajectory. Truth data is
%   converted from ECEF to NED using the reference coordinates stored in
%   EST_FILE. Position, velocity and acceleration residuals are then plotted.
%   The time vector is shifted so the first sample occurs at t=0 to match
%   Task 6. Figures are saved under OUTPUT_DIR using DATASET as part of the
%   filename.
%
%   Usage:
%       task7_ned_residuals_plot('fused_results.mat', 'STATE_X001.txt', ...
%           'IMU_X001_GNSS_X001', get_results_dir())
%
%   This MATLAB function mirrors the intended behaviour of the Python
%   counterpart ``task7_ned_residuals_plot.py``.

if nargin < 4 || isempty(output_dir)
    output_dir = get_results_dir();
end
if ~exist(output_dir, 'dir'); mkdir(output_dir); end

[t_est, pos_est, vel_est, acc_est, ref_lat, ref_lon, ref_r0] = load_est_ned(est_file);
[t_truth, pos_truth, vel_truth, acc_truth] = load_truth_ned(truth_file, ref_lat, ref_lon, ref_r0);

% Align time to start at zero for comparison with Task 6
t_rel = t_est - t_est(1);

pos_truth_i = interp1(t_truth, pos_truth, t_est, 'linear', 'extrap');
vel_truth_i = interp1(t_truth, vel_truth, t_est, 'linear', 'extrap');
acc_truth_i = interp1(t_truth, acc_truth, t_est, 'linear', 'extrap');

[res_pos, res_vel, res_acc] = compute_residuals(t_rel, pos_est, vel_est, pos_truth_i, vel_truth_i);

plot_residuals(t_rel, res_pos, res_vel, res_acc, dataset, output_dir);

end

% -------------------------------------------------------------------------
function [t, pos_ned, vel_ned, acc_ned, lat, lon, r0] = load_est_ned(file)
%LOAD_EST_NED  Load estimator output in the NED frame.

f = string(file);
if endsWith(f, '.npz')
    d = py.numpy.load(f);
    t = double(d{'time_s'});
    if isKey(d, 'pos_ned_m')
        pos_ned = double(d{'pos_ned_m'});
        vel_ned = double(d{'vel_ned_ms'});
    else
        pos_ecef = double(d{'pos_ecef_m'});
        vel_ecef = double(d{'vel_ecef_ms'});
    end
    if isKey(d, 'acc_ned_ms2')
        acc_ned = double(d{'acc_ned_ms2'});
    else
        acc_ned = [];
    end
    if isKey(d,'ref_lat_rad'); lat = double(d{'ref_lat_rad'}); else; lat = double(d{'ref_lat'}); end
    if isKey(d,'ref_lon_rad'); lon = double(d{'ref_lon_rad'}); else; lon = double(d{'ref_lon'}); end
    if isKey(d,'ref_r0_m'); r0 = double(d{'ref_r0_m'}); else; r0 = double(d{'ref_r0'}); end
else
    S = load(f);
    if isfield(S,'time_s'); t = S.time_s(:); else; t = S.time(:); end
    if isfield(S,'pos_ned_m')
        pos_ned = S.pos_ned_m;
        vel_ned = S.vel_ned_ms;
    else
        pos_ecef = S.pos_ecef_m;
        vel_ecef = S.vel_ecef_ms;
    end
    if isfield(S,'acc_ned_ms2'); acc_ned = S.acc_ned_ms2; else; acc_ned = []; end
    if isfield(S,'ref_lat_rad'); lat = S.ref_lat_rad; elseif isfield(S,'ref_lat'); lat = S.ref_lat; else; lat = 0; end
    if isfield(S,'ref_lon_rad'); lon = S.ref_lon_rad; elseif isfield(S,'ref_lon'); lon = S.ref_lon; else; lon = 0; end
    if isfield(S,'ref_r0_m'); r0 = S.ref_r0_m; elseif isfield(S,'ref_r0'); r0 = S.ref_r0; else; r0 = [0 0 0]; end
end

if ~exist('pos_ned','var') || isempty(pos_ned)
    C = compute_C_ECEF_to_NED(lat, lon);
    pos_ned = (C * (pos_ecef' - r0)).';
    vel_ned = (C * vel_ecef.').';
end

if isempty(acc_ned)
    dt = mean(diff(t));
    acc_ned = [zeros(1,3); diff(vel_ned)./diff(t)];
end
end

% -------------------------------------------------------------------------
function [t, pos_ned, vel_ned, acc_ned] = load_truth_ned(file, lat, lon, r0)
%LOAD_TRUTH_NED  Load truth trajectory and convert to NED frame.

f = string(file);
if endsWith(f,'.npz')
    d = py.numpy.load(f);
    t = double(d{'time_s'});
    pos_ecef = double(d{'pos_ecef_m'});
    vel_ecef = double(d{'vel_ecef_ms'});
    if isKey(d,'acc_ecef_ms2')
        acc_ecef = double(d{'acc_ecef_ms2'});
    else
        dt = mean(diff(t));
        acc_ecef = [zeros(1,3); diff(vel_ecef)./diff(t)];
    end
else
    raw = read_state_file(f);
    t = raw(:,1);
    pos_ecef = raw(:,2:4);
    vel_ecef = raw(:,5:7);
    acc_ecef = [zeros(1,3); diff(vel_ecef)./diff(t)];
end
C = compute_C_ECEF_to_NED(lat, lon);
pos_ned = (C * (pos_ecef' - r0)).';
vel_ned = (C * vel_ecef.').';
acc_ned = (C * acc_ecef.').';
end

% -------------------------------------------------------------------------
function [res_pos, res_vel, res_acc] = compute_residuals(t, est_pos, est_vel, truth_pos, truth_vel)
%COMPUTE_RESIDUALS Return position, velocity and acceleration residuals.
res_pos = est_pos - truth_pos;
res_vel = est_vel - truth_vel;
acc_est = [zeros(1,3); diff(est_vel)./diff(t)];
acc_truth = [zeros(1,3); diff(truth_vel)./diff(t)];
res_acc = acc_est - acc_truth;
end

% -------------------------------------------------------------------------
function plot_residuals(t, res_pos, res_vel, res_acc, dataset, out_dir)
%PLOT_RESIDUALS Plot residual components and norms in NED frame.
labels = {'North','East','Down'};
fig = figure('Visible','off','Position',[100 100 900 700]);
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
sgtitle(sprintf('%s Task 7 NED Residuals', dataset));
set(fig,'PaperPositionMode','auto');
pdf = fullfile(out_dir, sprintf('%s_task7_ned_residuals.pdf', dataset));
png = fullfile(out_dir, sprintf('%s_task7_ned_residuals.png', dataset));
print(fig, pdf, '-dpdf', '-bestfit');
print(fig, png, '-dpng', '-bestfit');
close(fig);

fig = figure('Visible','off');
plot(t, vecnorm(res_pos,2,2), 'DisplayName','|pos|'); hold on;
plot(t, vecnorm(res_vel,2,2), 'DisplayName','|vel|');
plot(t, vecnorm(res_acc,2,2), 'DisplayName','|acc|');
xlabel('Time [s]'); ylabel('Residual Norm'); legend; grid on;
set(fig,'PaperPositionMode','auto');
pdfn = fullfile(out_dir, sprintf('%s_task7_ned_residual_norms.pdf', dataset));
pngn = fullfile(out_dir, sprintf('%s_task7_ned_residual_norms.png', dataset));
print(fig, pdfn, '-dpdf', '-bestfit');
print(fig, pngn, '-dpng', '-bestfit');
close(fig);
end
