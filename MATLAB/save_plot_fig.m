function save_plot_fig(fig, filename)
%SAVE_PLOT_FIG  Save figure in MATLAB ``.fig`` format.
%   SAVE_PLOT_FIG(FIG, FILENAME) writes the given figure to *FILENAME*
%   using MATLAB's :func:`savefig` so it can be reopened later.  This
%   mirrors the Python ``save_plot_fig`` helper.

    if nargin < 1 || isempty(fig)
        fig = gcf; %#ok<GFLD> default to current figure if none provided
    end
    if nargin < 2 || isempty(filename)
        error('A filename must be provided');
    end

    savefig(fig, filename);
end
