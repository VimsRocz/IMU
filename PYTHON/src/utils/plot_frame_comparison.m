function plot_frame_comparison(t, data_sets, labels, frame_name, out_prefix, cfg)
%PLOT_FRAME_COMPARISON Plot 3-axis data in either 3-subplot or 1-panel mixed form.
%   t           : time vector (Nx1)
%   data_sets   : cell array of M×N or N×3 datasets (each dataset rows are axes)
%   labels      : cell array of legend strings
%   frame_name  : 'NED','ECEF','BODY','Mixed'
%   out_prefix  : full path prefix, e.g. results/.../RUNID_taskX
%   cfg         : struct with plotting policy. PNG output is saved via
%                 ``save_png_checked`` ensuring 1800×1200 px at 200 DPI.

if nargin < 6 || isempty(cfg)
    cfg = default_cfg();
end

visibleFlag = 'off';
try
    if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
        visibleFlag = 'on';
    end
catch
end

% Create figure with required dimensions
fig = figure('Visible', visibleFlag, 'Units','pixels', 'Position',[100 100 1800 1200]);

if strcmpi(frame_name,'Mixed')
    % single axes
    hold on; grid on;
    for i = 1:numel(data_sets)
        D = data_sets{i};
        % assume D is 3×N
        plot(t, D(1,:), '-', 'LineWidth', 1.5);
        plot(t, D(2,:), '--', 'LineWidth', 1.5);
        plot(t, D(3,:), ':', 'LineWidth', 1.5);
    end
    legend(labels,'Location','northoutside','Orientation','horizontal');
    xlabel('Time [s]'); ylabel(sprintf('%s axes',frame_name));
    title([frame_name ' Comparison']);
    hold off;
else
    % three stacked subplots
    axisLabels = get_axis_labels(frame_name);
    lines = gobjects(numel(data_sets),1);
    for ax = 1:3
        subplot(3,1,ax); hold on; grid on;
        for i = 1:numel(data_sets)
            D = data_sets{i};
            h = plot(t, D(ax,:), 'LineWidth', 1.5, 'DisplayName', labels{i});
            if ax == 1
                lines(i) = h;
            end
        end
        ylabel(sprintf('%s [m]', axisLabels{ax}));
        if ax==1
            title([frame_name ' Comparison']);
        end
        if ax==3
            xlabel('Time [s]');
        end
    end
    legend(lines, labels, 'Orientation','horizontal','Location','northoutside');
end

set(findall(fig,'-property','FontSize'),'FontSize',12);

% Save PNG
fname = sprintf('%s_%s.png', out_prefix, lower(frame_name));
if cfg.plots.save_png
    save_png_checked(fig, fname);
end
close(fig);
end

function labels = get_axis_labels(frame_name)
%GET_AXIS_LABELS Return axis labels for frames
switch upper(frame_name)
    case 'NED'
        labels = {'North','East','Down'};
    case 'ECEF'
        labels = {'X','Y','Z'};
    case 'BODY'
        labels = {'X','Y','Z'};
    otherwise
        labels = {'Axis1','Axis2','Axis3'};
end
end
