function save_all_figs_fig_only(outDir, prefix)
% Save every open MATLAB figure to native .fig in outDir (fig-only).
% Usage: save_all_figs_fig_only(fullfile(results_dir,'task5'), 'IMU_X002_GNSS_X002_TRIAD_task5')

if nargin < 1 || isempty(outDir), outDir = pwd; end
if nargin < 2, prefix = ''; end
if ~exist(outDir, 'dir'), mkdir(outDir); end

figs = findobj('Type','figure');
for k = 1:numel(figs)
    f = figs(k);
    % Build a stable name
    ttl = '';
    try
        ax = get(f, 'CurrentAxes');
        if ~isempty(ax)
            t = get(ax, 'Title');
            ttl = get(t, 'String');
        end
    catch
        ttl = '';
    end
    if iscell(ttl), ttl = strjoin(ttl, '_'); end
    if isempty(ttl), ttl = sprintf('figure_%d', double(f.Number)); end
    ttl = regexprep(ttl, '\s+|[/:\\]', '_');

    if ~isempty(prefix)
        name = sprintf('%s_%s.fig', prefix, ttl);
    else
        name = sprintf('%s.fig', ttl);
    end
    savefig(f, fullfile(outDir, name)); % native interactive figure
end
end

