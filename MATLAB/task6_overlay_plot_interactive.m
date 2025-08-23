function pdf_path = task6_overlay_plot_interactive(est_file, truth_file, method, frame, dataset, output_dir, debug)
%TASK6_OVERLAY_PLOT_INTERACTIVE  Create interactive overlay plot with enhanced features.
%
%   pdf_path = TASK6_OVERLAY_PLOT_INTERACTIVE(est_file, truth_file, method, frame,
%   dataset, output_dir, debug) creates an enhanced version of the Task 6
%   overlay plot with interactive features including zoom, pan, data cursors,
%   and improved layout consistent with the Python version.
%   
%   This function creates a 3x3 subplot layout (instead of the original 4x1)
%   to match the Python implementation, with interactive features enabled
%   for better data exploration.

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

% Load estimate and truth data
[t_est, pos_est, vel_est, acc_est] = load_estimate(est_file, frame);
[t_truth, pos_truth, vel_truth, acc_truth] = load_truth(truth_file, frame);

if debug
    print_debug_info(t_est, vel_est, t_truth, vel_truth);
    if numel(t_est) ~= numel(t_truth)
        fprintf('WARNING: time vector lengths differ (est %d, truth %d)\n', ...
            numel(t_est), numel(t_truth));
    end
end

% Interpolate truth data to estimate timeline
pos_truth_i = interp1(t_truth, pos_truth, t_est, 'linear', 'extrap');
vel_truth_i = interp1(t_truth, vel_truth, t_est, 'linear', 'extrap');
acc_truth_i = interp1(t_truth, acc_truth, t_est, 'linear', 'extrap');

% Ensure all arrays have the same length
[t_est, pos_est, vel_est, acc_est, pos_truth_i, vel_truth_i, acc_truth_i] = ...
    ensure_equal_length(t_est, pos_est, vel_est, acc_est, pos_truth_i, vel_truth_i, acc_truth_i);

% Create interactive overlay plot (position, velocity only; no acceleration)
pdf_path = plot_overlay_interactive_3x3(t_est, pos_est, vel_est, acc_est, ...
    pos_truth_i, vel_truth_i, acc_truth_i, frame, method, dataset, output_dir);

% Generate RMSE plot (position, velocity only; omit acceleration)
plot_rmse_interactive(t_est, pos_est, vel_est, acc_est, pos_truth_i, ...
    vel_truth_i, acc_truth_i, frame, method, dataset, output_dir);

end

% -------------------------------------------------------------------------
function pdf_path = plot_overlay_interactive_3x3(t_est, pos_est, vel_est, ~, ...
    pos_truth, vel_truth, ~, frame, method, dataset, out_dir)
%PLOT_OVERLAY_INTERACTIVE_3X3 Create interactive 3x3 overlay plot.
%   Creates a 3x3 subplot layout matching the Python implementation with
%   interactive features enabled for better data exploration.

% Define axis labels based on coordinate frame
if strcmpi(frame, 'NED')
    labels = {'ΔN [m]', 'ΔE [m]', 'ΔD [m]'};
    frame_title = 'NED';
elseif strcmpi(frame, 'ECEF')
    labels = {'X [m]', 'Y [m]', 'Z [m]'};
    frame_title = 'ECEF';
else % Body frame
    labels = {'X [m]', 'Y [m]', 'Z [m]'};
    frame_title = 'Body';
end

% Create figure with enhanced properties for interactivity
f = figure('Name', sprintf('%s Task 6 Interactive - %s (%s)', dataset, method, frame_title), ...
          'NumberTitle', 'off', ...
          'Position', [100 100 1200 800], ...
          'Color', 'white', ...
          'MenuBar', 'figure', ...
          'ToolBar', 'figure');

% Define data types and units for subplots (no acceleration)
data_types = {'Position', 'Velocity'};
units = {'[m]', '[m/s]'};
data_arrays = {pos_est, vel_est};
truth_arrays = {pos_truth, vel_truth};

% Color scheme matching Python version
colors.truth = [0, 0, 0];           % black
colors.fused = [0.1216, 0.4667, 0.7059];  % tab:blue
colors.measurements = [0.1725, 0.6275, 0.1725]; % tab:green (for consistency)

% Create 3x3 subplot layout
for row = 1:2
    for col = 1:3
        subplot(3, 3, (row-1)*3 + col);
        hold on; grid on;
        
        % Get data for this subplot
        current_data = data_arrays{row};
        current_truth = truth_arrays{row};
        
        % Plot truth data (black solid line)
        h_truth = plot(t_est, current_truth(:, col), '-', ...
                      'Color', colors.truth, ...
                      'LineWidth', 2, ...
                      'DisplayName', 'Truth');
        
        % Plot fused estimate (blue dotted line)
        h_fused = plot(t_est, current_data(:, col), ':', ...
                      'Color', colors.fused, ...
                      'LineWidth', 2, ...
                      'DisplayName', sprintf('Fused (%s)', method));
        
        % Set subplot title and labels
        if row == 1
            title(sprintf('%s %s %s', data_types{row}, labels{col}, units{row}), ...
                  'FontWeight', 'bold');
        else
            title(sprintf('%s %s %s', data_types{row}, labels{col}, units{row}));
        end
        
        % Add axis labels
        if row == 2  % Bottom row
            xlabel('Time [s]');
        end
        if col == 1  % Left column
            ylabel(sprintf('%s %s', data_types{row}, units{row}));
        end
        
        % Add legend only to the first subplot
        if row == 1 && col == 1
            legend([h_truth, h_fused], 'Location', 'northeast', ...
                   'FontSize', 10, 'Box', 'on');
        end
        
        % Enable interactive features
        zoom on;
        pan on;
        
        % Add data cursor mode for detailed inspection
        dcm = datacursormode(f);
        set(dcm, 'Enable', 'on', 'DisplayStyle', 'window');
        
        % Custom datatip function for better information display
        set(dcm, 'UpdateFcn', @(obj, event_obj) custom_datatip(obj, event_obj, ...
            data_types{row}, labels{col}, units{row}));
    end
end

% Add overall title
sgtitle(sprintf('%s Task 6 Interactive - %s (%s Frame)', dataset, method, frame_title), ...
        'FontSize', 16, 'FontWeight', 'bold');

% Enable advanced interactive features
set(f, 'WindowButtonDownFcn', @figure_button_down);
set(f, 'KeyPressFcn', @figure_key_press);

% Add interactive toolbar with custom buttons
add_custom_toolbar(f);

% Save the figure in multiple formats
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

run_id = sprintf('%s_%s', dataset, method);
base_filename = sprintf('%s_task6_interactive_overlay_%s', run_id, lower(frame));

% Save as MATLAB figure file for full interactivity
fig_path = fullfile(out_dir, [base_filename '.fig']);
savefig(f, fig_path);
fprintf('Saved interactive MATLAB figure: %s\n', fig_path);

% Save as PDF (static)
pdf_path = fullfile(out_dir, [base_filename '.pdf']);
print(f, pdf_path, '-dpdf', '-bestfit');
fprintf('Saved PDF: %s\n', pdf_path);

% Save as PNG (static)
png_path = fullfile(out_dir, [base_filename '.png']);
print(f, png_path, '-dpng', '-r300');
fprintf('Saved PNG: %s\n', png_path);

% Keep figure open for interaction (comment out to close automatically)
% close(f);

% Display interactive instructions
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('INTERACTIVE FEATURES ENABLED\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('✓ Zoom: Use zoom tool or mouse scroll wheel\n');
fprintf('✓ Pan: Use pan tool or click and drag\n');
fprintf('✓ Data cursor: Click on data points for exact values\n');
fprintf('✓ Reset view: Double-click on subplot\n');
fprintf('✓ Full screen: Press F key\n');
fprintf('✓ Grid toggle: Press G key\n');
fprintf('✓ Figure saved as: %s\n', fig_path);
fprintf('%s\n', repmat('=', 1, 60));

end

% -------------------------------------------------------------------------
function output_txt = custom_datatip(obj, event_obj, data_type, axis_label, units)
%CUSTOM_DATATIP Custom datatip function for enhanced information display.

pos = get(event_obj, 'Position');
target = get(event_obj, 'Target');
display_name = get(target, 'DisplayName');

output_txt = {
    sprintf('%s %s', data_type, axis_label), ...
    sprintf('Time: %.3f s', pos(1)), ...
    sprintf('Value: %.6f %s', pos(2), units), ...
    sprintf('Series: %s', display_name)
};

end

% -------------------------------------------------------------------------
function figure_button_down(src, ~)
%FIGURE_BUTTON_DOWN Handle mouse clicks for enhanced interaction.

click_type = get(src, 'SelectionType');
if strcmp(click_type, 'open')  % Double-click
    % Reset zoom/pan on double-click
    zoom(src, 'out');
    zoom(src, 'reset');
end

end

% -------------------------------------------------------------------------
function figure_key_press(src, event)
%FIGURE_KEY_PRESS Handle keyboard shortcuts for enhanced interaction.

switch event.Key
    case 'f'
        % Toggle fullscreen
        if strcmp(get(src, 'WindowState'), 'maximized')
            set(src, 'WindowState', 'normal');
        else
            set(src, 'WindowState', 'maximized');
        end
    case 'g'
        % Toggle grid on all subplots
        children = get(src, 'Children');
        for i = 1:length(children)
            if strcmp(get(children(i), 'Type'), 'axes')
                grid(children(i), 'toggle');
            end
        end
    case 'r'
        % Reset all views
        zoom(src, 'out');
        zoom(src, 'reset');
end

end

% -------------------------------------------------------------------------
function add_custom_toolbar(fig_handle)
%ADD_CUSTOM_TOOLBAR Add custom toolbar buttons for enhanced functionality.

% Get the figure's toolbar
toolbar = findall(fig_handle, 'Type', 'uitoolbar');
if isempty(toolbar)
    toolbar = uitoolbar('Parent', fig_handle);
end

% Add custom separator and button for resetting views
uipushtool(toolbar, 'Separator', 'on', ...
          'ToolTipString', 'Reset All Views', ...
          'ClickedCallback', @(src, event) reset_all_views(fig_handle), ...
          'CData', create_reset_icon());

% Add button for toggling grid
uipushtool(toolbar, ...
          'ToolTipString', 'Toggle Grid', ...
          'ClickedCallback', @(src, event) toggle_all_grids(fig_handle), ...
          'CData', create_grid_icon());

end

% -------------------------------------------------------------------------
function reset_all_views(fig_handle)
%RESET_ALL_VIEWS Reset zoom/pan for all subplots.

zoom(fig_handle, 'out');
zoom(fig_handle, 'reset');

end

% -------------------------------------------------------------------------
function toggle_all_grids(fig_handle)
%TOGGLE_ALL_GRIDS Toggle grid display for all subplots.

children = get(fig_handle, 'Children');
for i = 1:length(children)
    if strcmp(get(children(i), 'Type'), 'axes')
        grid(children(i), 'toggle');
    end
end

end

% -------------------------------------------------------------------------
function icon_data = create_reset_icon()
%CREATE_RESET_ICON Create icon data for reset button.

% Simple 16x16 icon (curved arrow for reset)
icon_data = ones(16, 16, 3) * 0.8;  % Light gray background
% Add some simple pattern (this is a placeholder - could be improved)
icon_data(8:10, 4:12, :) = 0.2;  % Dark line
icon_data = repmat(icon_data, [1, 1, 1]);

end

% -------------------------------------------------------------------------
function icon_data = create_grid_icon()
%CREATE_GRID_ICON Create icon data for grid toggle button.

% Simple 16x16 grid icon
icon_data = ones(16, 16, 3) * 0.8;  % Light gray background
% Add grid lines
icon_data(4:4:16, :, :) = 0.2;  % Horizontal lines
icon_data(:, 4:4:16, :) = 0.2;  % Vertical lines

end

% -------------------------------------------------------------------------
function pdf_path = plot_rmse_interactive(t, pos_est, vel_est, ~, pos_truth, vel_truth, ~, frame, method, dataset, out_dir)
%PLOT_RMSE_INTERACTIVE Plot interactive RMSE (pos, vel only).

% Calculate error magnitudes
pos_err = vecnorm(pos_est - pos_truth, 2, 2);
vel_err = vecnorm(vel_est - vel_truth, 2, 2);

% Calculate RMSE values
rmse_pos = sqrt(mean(pos_err.^2));
rmse_vel = sqrt(mean(vel_err.^2));

% Create interactive figure
f = figure('Name', sprintf('%s RMSE Analysis - %s (%s)', dataset, method, frame), ...
          'NumberTitle', 'off', ...
          'Position', [150 150 800 600], ...
          'Color', 'white');

hold on; grid on;

% Plot error time series with enhanced styling
h1 = plot(t, pos_err, 'LineWidth', 2, 'DisplayName', ...
         sprintf('Position RMSE: %.3f m', rmse_pos));
h2 = plot(t, vel_err, 'LineWidth', 2, 'DisplayName', ...
         sprintf('Velocity RMSE: %.3f m/s', rmse_vel));
% No acceleration RMSE in truth; omit third series

xlabel('Time [s]', 'FontSize', 12);
ylabel('Error Magnitude', 'FontSize', 12);
title(sprintf('%s RMSE Analysis - %s (%s Frame)', dataset, method, upper(frame)), ...
      'FontSize', 14, 'FontWeight', 'bold');

legend('Location', 'northeast', 'FontSize', 10);

% Enable interactive features
zoom on; pan on;
dcm = datacursormode(f);
set(dcm, 'Enable', 'on', 'DisplayStyle', 'window');

% Save figure
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

base_filename = sprintf('%s_%s_Task6_%s_RMSE_Interactive', dataset, method, upper(frame));
fig_path = fullfile(out_dir, [base_filename '.fig']);
pdf_path = fullfile(out_dir, [base_filename '.pdf']);
png_path = fullfile(out_dir, [base_filename '.png']);

savefig(f, fig_path);
print(f, pdf_path, '-dpdf', '-bestfit');
print(f, png_path, '-dpng', '-r300');

fprintf('Saved interactive RMSE figure: %s\n', fig_path);
fprintf('Saved RMSE PDF: %s\n', pdf_path);

end

% -------------------------------------------------------------------------
% Keep existing helper functions from original file
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
%PRINT_DEBUG_INFO Display diagnostic information.

fprintf('Length of fused velocity: %s\n', mat2str(size(vel_est)));
fprintf('Length of truth velocity: %s\n', mat2str(size(vel_truth)));
fprintf('Fused time range: %.3f to %.3f\n', t_est(1), t_est(end));
fprintf('Truth time range: %.3f to %.3f\n', t_truth(1), t_truth(end));

end

% -------------------------------------------------------------------------
function [t_est, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth] = ...
    ensure_equal_length(t_est, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth)
%ENSURE_EQUAL_LENGTH Truncate arrays to the common minimum length.

if size(pos_est,1) ~= size(pos_truth,1)
    n = min(size(pos_est,1), size(pos_truth,1));
    warning('Length mismatch after interpolation (est %d, truth %d); truncating to %d', ...
            size(pos_est,1), size(pos_truth,1), n);
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
