function plot_state_grid(t, pos, vel, acc, frame, tag, outdir, legendEntries)
%PLOT_STATE_GRID 3x3 pop-up of Position/Velocity/Acceleration for N/E/D.
%   PLOT_STATE_GRID(T, POS, VEL, ACC, FRAME, TAG, OUTDIR, LEGENDENTRIES)
%   displays a 3×3 grid of plots showing position, velocity and
%   acceleration components over time. FRAME is a string identifying the
%   coordinate frame (e.g. 'NED'), TAG is used for the figure name and file
%   prefix. OUTDIR is where plots are saved. LEGENDENTRIES is optional and
%   allows custom legend labels when multiple series are provided.
%
%   POS, VEL and ACC may be either N×3 arrays or cell arrays of such arrays
%   to overlay multiple series. Each array must have the same number of rows
%   as T.

if nargin < 8
    legendEntries = {};
end

figure('Name', sprintf('%s %s', frame, tag), 'NumberTitle','off', 'Visible','on');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
rows = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
cols = {'North','East','Down'};

isCell = iscell(pos);

for r = 1:3
    for c = 1:3
        nexttile; hold on; grid on;
        switch r
            case 1
                series = pos;
                ylab = rows{1};
            case 2
                series = vel;
                ylab = rows{2};
            case 3
                series = acc;
                ylab = rows{3};
        end
        if isCell
            for j = 1:numel(series)
                plot(t, series{j}(:,c), 'LineWidth',1);
            end
        else
            plot(t, series(:,c), 'LineWidth',1);
        end
        if r == 1
            title(cols{c});
        end
        if c == 1
            ylabel(ylab);
        end
        if r == 3
            xlabel('Time [s]');
        end
        if ~isempty(legendEntries) && c == 2 && r == 1
            legend(legendEntries, 'Location', 'best'); legend boxoff;
        end
    end
end

if nargin >= 7 && ~isempty(outdir)
    if ~exist(outdir,'dir')
        mkdir(outdir);
    end
    base = sprintf('%s_%s_state_%s', tag, frame, datestr(now,'yyyymmdd_HHMMSS'));
    exportgraphics(gcf, fullfile(outdir, [base '.pdf']), 'ContentType','vector');
    saveas(gcf, fullfile(outdir, [base '.png']));
end
end
