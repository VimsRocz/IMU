function rebuild_and_save_fig(mat_path, fig_out_path)
%REBUILD_AND_SAVE_FIG Rebuild a line plot from a .mat file and save .fig.
%   rebuild_and_save_fig(mat_path, fig_out_path) loads plotting data saved
%   from Python and recreates the figure, exporting it as a MATLAB .fig.
%
%   The .mat file schema is defined in python/utils/save_plot_all.py.

    S = load(mat_path);
    f = figure('Color','w');
    ax = axes('Parent', f);
    hold(ax, 'on');

    n = S.n_lines;
    for i = 1:n
        xi = S.line_x{1,i};
        yi = S.line_y{1,i};
        lbl = S.line_label{i};
        col = S.line_color{i};
        ls  = S.line_ls{i};
        lw  = S.line_lw(i);
        mk  = S.line_marker{i};
        mks = S.line_mks(i);
        p = plot(ax, xi, yi, 'LineStyle',ls, 'LineWidth',lw, ...
                 'Marker',mk, 'MarkerSize',mks);
        if ischar(col)
            try, set(p, 'Color', col); end
        elseif isnumeric(col) && numel(col) >= 3
            set(p, 'Color', col(1:3));
        end
        if ~isempty(lbl), set(p, 'DisplayName', lbl); end
    end

    title(ax, S.title);
    xlabel(ax, S.xlabel);
    ylabel(ax, S.ylabel);
    if isfield(S, 'legend_visible') && S.legend_visible ~= 0
        if isfield(S, 'legend_loc') && ~isempty(S.legend_loc)
            legend(ax, 'show', 'Location', S.legend_loc);
        else
            legend(ax, 'show');
        end
    end

    savefig(f, fig_out_path);
    fprintf('Saved .fig -> %s\n', fig_out_path);
end
