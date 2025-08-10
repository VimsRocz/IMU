function save_plot_all(h, basepath, formats)
%SAVE_PLOT_ALL Save MATLAB figure to multiple formats.
%   save_plot_all(h, basepath, formats) saves figure handle h (or gcf) to
%   the extensions listed in cell array ``formats``. If ``formats`` is
%   omitted, only a ``.fig`` is written.
%
%   Usage:
%       save_plot_all(gcf, 'results/demo', {'.fig','.png','.pdf'})
%
%   See also: SAVEFIG, EXPORTGRAPHICS

    if nargin < 1 || isempty(h), h = gcf; end
    if nargin < 2 || isempty(basepath)
        error('basepath required');
    end
    if nargin < 3 || isempty(formats), formats = {'.fig'}; end

    [outdir,~,~] = fileparts(basepath);
    if ~isempty(outdir) && ~exist(outdir,'dir')
        mkdir(outdir);
    end
    set(h,'Color','w');
    drawnow;

    for k = 1:numel(formats)
        ext = lower(formats{k});
        outfile = [basepath, ext];
        switch ext
            case '.fig'
                savefig(h, outfile);
            case '.png'
                exportgraphics(h, outfile, 'Resolution',300);
            case {'.pdf', '.svg'}
                exportgraphics(h, outfile, 'ContentType','vector');
            otherwise
                warning('Unsupported format: %s', ext);
                continue;
        end
        fprintf('Saved -> %s\n', outfile);
    end
end
