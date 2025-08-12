function save_plot_fig(fig, filename)
%SAVE_PLOT_FIG  Save a figure in MATLAB ``.fig`` format.
%   SAVE_PLOT_FIG(FIG, FILENAME) writes the handle FIG to FILENAME in
%   ``.fig`` format, creating directories as needed.  If FIG is empty the
%   current figure ``gcf`` is used.

    if nargin < 1 || isempty(fig)
        fig = gcf;
    end
    if nargin < 2 || isempty(filename)
        error('save_plot_fig:missingFile', 'Output filename must be specified');
    end

    out_dir = fileparts(filename);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    savefig(fig, filename);
    fprintf('Saved figure to %s\n', filename);
end
