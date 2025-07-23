function pdf_path = task6_overlay_plot(est_file, truth_file, method, frame, dataset, output_dir, debug)
%TASK6_OVERLAY_PLOT  Overlay fused estimate with ground truth.
%
%   pdf_path = TASK6_OVERLAY_PLOT(est_file, truth_file, method, frame,
%   dataset, output_dir, debug) mirrors the functionality of the Python
%   script ``task6_overlay_plot.py``.  ``est_file`` and ``truth_file`` are
%   ``.npz`` or ``.txt`` files containing the fused estimator output and
%   ground truth respectively. ``frame`` is either ``'ECEF'`` or ``'NED'``.
%   The interpolated truth is overlaid on the estimate for position,
%   velocity and acceleration and the figure saved under ``output_dir`` with
%   a name ``<dataset>_<method>_Task6_<frame>_Overlay``.

if nargin < 4 || isempty(frame)
    frame = 'ECEF';
end
if nargin < 5
    dataset = 'DATASET';
end
if nargin < 6 || isempty(output_dir)
    output_dir = 'results';
end
if nargin < 7
    debug = false;
end

[t_est, pos_est, vel_est, acc_est] = load_estimate(est_file, frame);
[t_truth, pos_truth, vel_truth, acc_truth] = load_truth(truth_file, frame);

if debug
    print_debug_info(t_est, vel_est, t_truth, vel_truth);
    if numel(t_est) ~= numel(t_truth)
        fprintf('WARNING: time vector lengths differ (est %d, truth %d)\n', ...
            numel(t_est), numel(t_truth));
    end
end

pos_truth_i = interp1(t_truth, pos_truth, t_est, 'linear', 'extrap');
vel_truth_i = interp1(t_truth, vel_truth, t_est, 'linear', 'extrap');
acc_truth_i = interp1(t_truth, acc_truth, t_est, 'linear', 'extrap');

pdf_path = plot_overlay(t_est, pos_est, vel_est, acc_est, pos_truth_i, ...
    vel_truth_i, acc_truth_i, frame, method, dataset, output_dir);

end

% -------------------------------------------------------------------------
function [t, pos, vel, acc] = load_truth(path, frame)
%LOAD_TRUTH Load truth data from STATE_X file or NPZ.

ppath = string(path);
if endsWith(ppath, '.npz')
    data = py.numpy.load(ppath);
    if strcmpi(frame, 'ECEF')
        pos = double(data{'pos_ecef_m'});
        vel = double(data{'vel_ecef_ms'});
    else
        pos = double(data{'pos_ned_m'});
        vel = double(data{'vel_ned_ms'});
    end
    t = double(data{'time_s'});
else
    raw = load(ppath);
    t = raw(:,2);
    if strcmpi(frame,'ECEF')
        pos = raw(:,3:5);
        vel = raw(:,5:8);
    else
        pos = raw(:,9:11);
        vel = raw(:,12:14);
    end
end
dt = mean(diff(t));
acc = gradient(gradient(pos)) / dt^2;
end

% -------------------------------------------------------------------------
function [t, pos, vel, acc] = load_estimate(path, frame)
%LOAD_ESTIMATE Load fused estimator result from NPZ.

ppath = string(path);
data = py.numpy.load(ppath);
t = double(data{'time_s'});
if strcmpi(frame,'ECEF')
    pos = double(data{'pos_ecef_m'});
    vel = double(data{'vel_ecef_ms'});
else
    pos = double(data{'pos_ned_m'});
    vel = double(data{'vel_ned_ms'});
end
if isKey(data, 'acc_ecef_ms2')
    acc = double(data{sprintf('acc_%s_ms2', lower(frame))});
else
    dt = mean(diff(t));
    acc = gradient(gradient(pos)) / dt^2;
end
end

% -------------------------------------------------------------------------
function print_debug_info(t_est, vel_est, t_truth, vel_truth)
%PRINT_DEBUG_INFO Display diagnostic information similar to Python version.

fprintf('Length of fused velocity: %s\n', mat2str(size(vel_est)));
fprintf('Length of truth velocity: %s\n', mat2str(size(vel_truth)));
fprintf('Fused time range: %.3f to %.3f\n', t_est(1), t_est(end));
fprintf('Truth time range: %.3f to %.3f\n', t_truth(1), t_truth(end));
disp('First 5 fused velocity X:'); disp(vel_est(1:min(5,end),1)');
disp('First 5 truth velocity X:'); disp(vel_truth(1:min(5,end),1)');
disp('Last 5 fused velocity X:'); disp(vel_est(max(1,end-4):end,1)');
disp('Last 5 truth velocity X:'); disp(vel_truth(max(1,end-4):end,1)');
dv_truth = diff(vel_truth(:,1));
dv_est = diff(vel_est(:,1));
fprintf('Max velocity jump in truth (X): %.3f\n', max(abs(dv_truth)));
fprintf('Max velocity jump in fused (X): %.3f\n', max(abs(dv_est)));
end

% -------------------------------------------------------------------------
function pdf_path = plot_overlay(t_est, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth, frame, method, dataset, out_dir)
%PLOT_OVERLAY Create overlay plot of fused vs truth.

labels = {'X','Y','Z'};
if strcmpi(frame,'NED')
    labels = {'N','E','D'};
end
colors = {[0.2157 0.4824 0.7216], [0.8941 0.1020 0.1098], [0.3010 0.6863 0.2902]};

f = figure('Visible','off','Position',[100 100 800 900]);
for ax = 1:3
    subplot(3,1,ax);
    data_est = {pos_est, vel_est, acc_est};
    data_truth = {pos_truth, vel_truth, acc_truth};
    plot(t_est, data_est{ax}(:,1), 'Color', colors{1}, 'DisplayName', ['Fused ' labels{1}]); hold on;
    plot(t_est, data_truth{ax}(:,1), '--', 'Color', colors{1}, 'DisplayName', ['Truth ' labels{1}]);
    plot(t_est, data_est{ax}(:,2), 'Color', colors{2}, 'DisplayName', ['Fused ' labels{2}]);
    plot(t_est, data_truth{ax}(:,2), '--', 'Color', colors{2}, 'DisplayName', ['Truth ' labels{2}]);
    plot(t_est, data_est{ax}(:,3), 'Color', colors{3}, 'DisplayName', ['Fused ' labels{3}]);
    plot(t_est, data_truth{ax}(:,3), '--', 'Color', colors{3}, 'DisplayName', ['Truth ' labels{3}]);
    switch ax
        case 1
            ylabel('Position [m]');
            title(sprintf('%s Task 6 Overlay \x2013 %s (%s frame)', dataset, method, upper(frame)));
        case 2
            ylabel('Velocity [m/s]');
        otherwise
            ylabel('Acceleration [m/s^2]');
            xlabel('Time [s]');
    end
    grid on;
    if ax==1
        legend('Location','northeastoutside');
    end
end
set(f,'PaperPositionMode','auto');
if ~exist(out_dir,'dir'); mkdir(out_dir); end
pdf_path = fullfile(out_dir, sprintf('%s_%s_Task6_%s_Overlay.pdf', dataset, method, upper(frame)));
png_path = fullfile(out_dir, sprintf('%s_%s_Task6_%s_Overlay.png', dataset, method, upper(frame)));
print(f, pdf_path, '-dpdf');
print(f, png_path, '-dpng');
close(f);
fprintf('Saved overlay figure to %s\n', pdf_path);
end

