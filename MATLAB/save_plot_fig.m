function save_plot_fig(fig, filename)
%SAVE_PLOT_FIG Save figure in MATLAB ``.fig`` format.
%   SAVE_PLOT_FIG(FIG, FILENAME) writes the figure handle FIG to
%   the path specified by FILENAME using MATLAB's ``savefig``.  The
%   directory is created if necessary.  This mirrors the Python helper
%   ``save_plot_fig`` in ``src/utils.py``.
%
%   Example:
%       save_plot_fig(gcf, 'results/my_figure.fig')

    arguments
        fig matlab.ui.Figure
        filename (1, :) char
    end

    out_dir = fileparts(filename);
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    savefig(fig, filename);
    fprintf('Saved figure to %s\n', filename);
end
