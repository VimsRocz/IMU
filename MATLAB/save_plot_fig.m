function save_plot_fig(fig, filename)
%SAVE_PLOT_FIG  Save a MATLAB figure to .fig ensuring parent directory exists.
%   SAVE_PLOT_FIG(FIG, FILENAME) writes the figure handle FIG to the
%   specified FILENAME using MATLAB's ``savefig``.  If the directory of
%   FILENAME does not exist it is created automatically.  The caller is
%   responsible for providing the full path including ``.fig`` extension.

    if nargin < 2 || isempty(filename)
        results_dir = get_results_dir();
        if ~exist(results_dir, 'dir')
            mkdir(results_dir);
        end
        filename = fullfile(results_dir, 'figure.fig');
    else
        [out_dir, ~, ~] = fileparts(filename);
        if ~isempty(out_dir) && ~exist(out_dir, 'dir')
            mkdir(out_dir);
        end
    end

    savefig(fig, filename);
end
