function pdf_path = task6_overlay_plot(est_file, truth_file, method, frame, dataset, output_dir, debug, interactive)
%TASK6_OVERLAY_PLOT  Overlay fused estimate with ground truth.
%
%   pdf_path = TASK6_OVERLAY_PLOT(est_file, truth_file, method, frame,
%   dataset, output_dir, debug, interactive) mirrors the functionality of the Python
%   script ``task6_overlay_plot.py``.  ``est_file`` and ``truth_file`` are
%   ``.npz`` or ``.txt`` files containing the fused estimator output and
%   ground truth respectively. ``frame`` is either ``'ECEF'`` or ``'NED'``.
%   The interpolated truth is overlaid on the estimate for position,
%   velocity and acceleration and the figure saved under
%   ``results/<run_id>/`` as ``<run_id>_task6_overlay_state_<frame>.pdf``
%   within the directory returned by ``get_results_dir()``.
%   ``run_id`` combines the dataset and method, e.g.,
%   ``IMU_X003_GNSS_X002_TRIAD``.
%
%   The ``interactive`` parameter (default true) enables enhanced 3x3 layout
%   with interactive features including zoom, pan, and data cursors.

if nargin < 4 || isempty(frame)
    frame = 'ECEF';
end
if nargin < 5
    dataset = 'DATASET';
end
if nargin < 6 || isempty(output_dir)
    output_dir = get_results_dir();
end
if nargin < 7
    debug = false;
end
if nargin < 8
    interactive = true;  % Default to interactive mode
end

% Use interactive version by default
if interactive
    try
        pdf_path = task6_overlay_plot_interactive(est_file, truth_file, method, ...
                                                 frame, dataset, output_dir, debug);
        return;
    catch ME
        warning('Interactive plotting failed: %s. Falling back to standard plot.', ME.message);
        interactive = false;
    end
end

% Original implementation (4x1 layout) for compatibility
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

[t_est, pos_est, vel_est, acc_est, pos_truth_i, vel_truth_i, acc_truth_i] = ...
    ensure_equal_length(t_est, pos_est, vel_est, acc_est, pos_truth_i, vel_truth_i, acc_truth_i);

pdf_path = plot_overlay_legacy(t_est, pos_est, vel_est, acc_est, pos_truth_i, ...
    vel_truth_i, acc_truth_i, frame, method, dataset, output_dir);

plot_rmse(t_est, pos_est, vel_est, acc_est, pos_truth_i, ...
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
    % Text truth files may include a comment header
    raw = read_state_file(ppath);
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
%LOAD_ESTIMATE Load fused estimator result from NPZ or MAT.
%   Covariance matrices (P or P_hist) may be absent in the files. In
%   line with the Python implementation, missing covariance is silently
%   ignored here.

ppath = string(path);
if endsWith(ppath,'.npz')
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
else
    S = load(ppath);
    if isfield(S,'time_s'); t = S.time_s(:); else; t = S.time(:); end
    if strcmpi(frame,'ECEF')
        pos = S.pos_ecef_m;
        vel = S.vel_ecef_ms;
        if isfield(S,'acc_ecef_ms2')
            acc = S.acc_ecef_ms2;
        else
            dt = mean(diff(t));
            acc = gradient(gradient(pos)) / dt^2;
        end
    else
        pos = S.pos_ned;
        vel = S.vel_ned;
        if isfield(S,'acc_ned_ms2')
            acc = S.acc_ned_ms2;
        else
            dt = mean(diff(t));
            acc = gradient(gradient(pos)) / dt^2;
        end
    end
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
function [t_est, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth] = ...
    ensure_equal_length(t_est, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth)
%ENSURE_EQUAL_LENGTH Truncate arrays to the common minimum length.
if size(pos_est,1) ~= size(pos_truth,1)
    n = min(size(pos_est,1), size(pos_truth,1));
    warning('Length mismatch after interpolation (est %d, truth %d); truncating to %d', size(pos_est,1), size(pos_truth,1), n);
    t_est = t_est(1:n);
    pos_est = pos_est(1:n,:);
    vel_est = vel_est(1:n,:);
    acc_est = acc_est(1:n,:);
    pos_truth = pos_truth(1:n,:);
    vel_truth = vel_truth(1:n,:);
    acc_truth = acc_truth(1:n,:);
end
assert(size(pos_est,1) == size(pos_truth,1));
end

% -------------------------------------------------------------------------
function pdf_path = plot_overlay_legacy(t_est, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth, frame, method, dataset, out_dir)
%PLOT_OVERLAY_LEGACY Original 4x1 overlay plot for compatibility.
%   The original 4x1 layout preserved for backward compatibility.

labels = {'X','Y','Z'};
if strcmpi(frame,'NED')
    labels = {'N','E','D'};
end
    ylabels = {'Position [m]','Velocity [m/s]'};

    f = figure('Visible','off','Position',[100 100 900 600]);
    data_est  = {pos_est,  vel_est};
    data_truth = {pos_truth, vel_truth};
    for row = 1:2
        for col = 1:3
            subplot(2,3,(row-1)*3+col); hold on;
            % Fused estimate in red with 'x' marker
            if row == 1 && col == 1
                plot(t_est, data_est{row}(:,col), 'r-', 'LineWidth',2, ...
                    'Marker','x', 'Markersize',4, 'DisplayName','Fused');
                plot(t_est, data_truth{row}(:,col), 'k:', ...
                    'Marker','*', 'Markersize',4, 'DisplayName','Truth');
            else
                plot(t_est, data_est{row}(:,col), 'r-', 'LineWidth',2, ...
                    'Marker','x', 'Markersize',4, 'HandleVisibility','off');
                plot(t_est, data_truth{row}(:,col), 'k:', ...
                    'Marker','*', 'Markersize',4, 'HandleVisibility','off');
            end
            grid on;
            if row == 1
                title(labels{col});
            end
            if col == 1
                ylabel(ylabels{row});
            end
            xlabel('Time [s]');
            if row == 1 && col == 1
                legend('Location','northeast', 'Box','on', 'FontSize',10);
            end
        end
    end
sgtitle(sprintf('%s Task 6 Overlay - %s (%s frame)', dataset, method, upper(frame)));
set(f,'PaperPositionMode','auto');
run_id = sprintf('%s_%s', dataset, method);
if ~exist(out_dir,'dir'); mkdir(out_dir); end
pdf_path = fullfile(out_dir, sprintf('%s_task6_overlay_state_%s.pdf', run_id, frame));
png_path = fullfile(out_dir, sprintf('%s_task6_overlay_state_%s.png', run_id, frame));
print(f, pdf_path, '-dpdf', '-bestfit');
print(f, png_path, '-dpng', '-bestfit');
close(f);
fprintf('Saved overlay figure to %s\n', pdf_path);
files = dir(fullfile(out_dir, sprintf('%s_task6_*.pdf', run_id)));
if ~isempty(files)
    fprintf('Files saved in %s\n', out_dir);
    for k=1:numel(files)
        fprintf(' - %s\n', files(k).name);
    end
end
end

% -------------------------------------------------------------------------
function pdf_path = plot_rmse(t, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth, frame, method, dataset, out_dir)
%PLOT_RMSE Plot total error magnitude and annotate RMSE values.

pos_err = vecnorm(pos_est - pos_truth, 2, 2);
vel_err = vecnorm(vel_est - vel_truth, 2, 2);
acc_err = vecnorm(acc_est - acc_truth, 2, 2);

rmse_pos = sqrt(mean(pos_err.^2));
rmse_vel = sqrt(mean(vel_err.^2));
rmse_acc = sqrt(mean(acc_err.^2));

f = figure('Visible','off','Position',[100 100 600 400]);
plot(t, pos_err, 'DisplayName', sprintf('Pos RMSE %.3f m', rmse_pos)); hold on;
plot(t, vel_err, 'DisplayName', sprintf('Vel RMSE %.3f m/s', rmse_vel));
plot(t, acc_err, 'DisplayName', sprintf('Acc RMSE %.3f m/s^2', rmse_acc));
xlabel('Time [s]');
ylabel('Error magnitude');
grid on;
legend('Location','northeast');
title(sprintf('%s Task 6 RMSE - %s (%s frame)', dataset, method, upper(frame)));
set(f,'PaperPositionMode','auto');
if ~exist(out_dir,'dir'); mkdir(out_dir); end
pdf_path = fullfile(out_dir, sprintf('%s_%s_Task6_%s_RMSE.pdf', dataset, method, upper(frame)));
png_path = fullfile(out_dir, sprintf('%s_%s_Task6_%s_RMSE.png', dataset, method, upper(frame)));
print(f, pdf_path, '-dpdf', '-bestfit');
print(f, png_path, '-dpng', '-bestfit');
close(f);
fprintf('Saved RMSE figure to %s\n', pdf_path);
files = dir(fullfile(out_dir, sprintf('%s_%s_Task6_*_RMSE.pdf', dataset, method)));
if ~isempty(files)
    fprintf('Files saved in %s\n', out_dir);
    for k=1:numel(files)
        fprintf(' - %s\n', files(k).name);
    end
end
end

