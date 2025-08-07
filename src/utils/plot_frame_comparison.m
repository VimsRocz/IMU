function plot_frame_comparison(t, data_sets, labels, frame_name, out_prefix)
%PLOT_FRAME_COMPARISON  Plot multiple 3-axis datasets against time.
%   PLOT_FRAME_COMPARISON(T, DATA_SETS, LABELS, FRAME_NAME, OUT_PREFIX)
%   creates comparison figures for the provided DATA_SETS.  Each element of
%   DATA_SETS must be a 3xN or Nx3 matrix representing the X/Y/Z components
%   over the time vector T.  LABELS supplies legend entries for each data
%   set.  FRAME_NAME selects the type of comparison: 'NED', 'ECEF', 'BODY'
%   produce three stacked subplots while 'Mixed' overlays all axes in a
%   single plot.  OUT_PREFIX is the full path prefix (including run ID and
%   task) to which the figure will be saved as both PDF and PNG.
%
%   The function uses MATLAB's default colour order.  Distinct line styles
%   and markers are assigned per dataset to maintain clarity without
%   hardcoding colours.

    if nargin < 5
        error('plot_frame_comparison:BadArgs', ...
              'Expected t, data\_sets, labels, frame\_name, out\_prefix');
    end

    n_sets = numel(data_sets);
    if n_sets ~= numel(labels)
        error('plot_frame_comparison:LabelMismatch', ...
              'DATA\_SETS and LABELS must have the same length');
    end

    % Ensure data are 3xN
    for k = 1:n_sets
        D = data_sets{k};
        if size(D,1) ~= 3 && size(D,2) == 3
            data_sets{k} = D.';
        elseif size(D,1) ~= 3
            error('plot_frame_comparison:BadShape', ...
                  'Each dataset must be 3xN or Nx3');
        end
    end

    line_styles = {'-','--',':','-.'};
    markers = {'o','s','d','^'};
    axes_labels = {'X','Y','Z'};
    switch upper(frame_name)
        case 'NED'
            axes_labels = {'North [m]','East [m]','Down [m]'};
        case 'ECEF'
            axes_labels = {'X [m]','Y [m]','Z [m]'};
        case 'BODY'
            axes_labels = {'bX [m]','bY [m]','bZ [m]'};
        otherwise
            % Mixed frame uses generic label later
    end

    if ~strcmpi(frame_name,'Mixed')
        fig = figure('Visible','off','Position',[100 100 800 800]);
        for i = 1:3
            ax = subplot(3,1,i); hold(ax,'on'); grid(ax,'on');
            for k = 1:n_sets
                ls = line_styles{mod(k-1,numel(line_styles))+1};
                plot(ax, t, data_sets{k}(i,:), ls, 'DisplayName', labels{k});
            end
            ylabel(ax, axes_labels{i});
            if i==3; xlabel(ax,'Time [s]'); end
            legend(ax,'Location','best');
        end
        sgtitle([frame_name ' Comparison']);
    else
        fig = figure('Visible','off','Position',[100 100 800 600]);
        ax = axes(fig); hold(ax,'on'); grid(ax,'on');
        for k = 1:n_sets
            ls = line_styles{mod(k-1,numel(line_styles))+1};
            for j = 1:3
                mk = markers{mod(j-1,numel(markers))+1};
                plot(ax, t, data_sets{k}(j,:), [ls mk], ...
                     'DisplayName', sprintf('%s %s', labels{k}, axes_labels{j}));
            end
        end
        xlabel(ax,'Time [s]'); ylabel(ax,'Value [m]');
        legend(ax,'Location','best');
        title(ax,[frame_name ' Comparison']);
    end

    if ~exist(fileparts(out_prefix),'dir')
        mkdir(fileparts(out_prefix));
    end
    saveas(fig, [out_prefix '_' frame_name '_comparison.pdf'], 'pdf');
    saveas(fig, [out_prefix '_' frame_name '_comparison.png'], 'png');
    close(fig);
end
