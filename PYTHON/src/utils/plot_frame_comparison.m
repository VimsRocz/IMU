function plot_frame_comparison(t, data_sets, labels, frame_name, out_prefix, cfg)
%PLOT_FRAME_COMPARISON Plot 3-axis data in either 3-subplot or 1-panel mixed form.
%   t           : time vector (Nx1)
%   data_sets   : cell array of M×N or N×3 datasets (each dataset rows are axes)
%   labels      : cell array of legend strings
%   frame_name  : 'NED','ECEF','BODY','Mixed'
%   out_prefix  : full path prefix, e.g. results/.../RUNID_taskX
%   cfg         : struct with plotting policy

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

% Create figure
figure('Visible', visibleFlag);
if strcmpi(frame_name,'Mixed')
    % single axes
    hold on; grid on;
    for i = 1:numel(data_sets)
        D = data_sets{i};
        % assume D is 3×N
        plot(t, D(1,:), '-'); plot(t, D(2,:), '--'); plot(t, D(3,:), ':');
    end
    legend(labels,'Location','best');
    xlabel('Time (s)'); ylabel(sprintf('%s axes',frame_name));
    title([frame_name ' Comparison']);
    hold off;
else
    % three stacked subplots
    for ax = 1:3
        subplot(3,1,ax); hold on; grid on;
        for i = 1:numel(data_sets)
            D = data_sets{i};
            plot(t, D(ax,:), 'DisplayName', labels{i});
        end
        ylabel(sprintf('%s_%d',frame_name,ax));
        if ax==1, title([frame_name ' Comparison']); end
        if ax==3, xlabel('Time (s)'); end
        if ax==1, legend('show','Location','best'); end
        hold off;
    end
end
% Save PNG
fname = [out_prefix '_' frame_name '_comparison'];
if cfg.plots.save_png
    saveas(gcf, [fname '.png'], 'png');
end
close;
end
