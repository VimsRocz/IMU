function save_plot_png(fig, out_dir, stem)
%SAVE_PLOT_PNG Save figure as PNG with timestamp and latest copy
if nargin < 1 || isempty(fig); fig = gcf; end
if ~exist(out_dir,'dir'); mkdir(out_dir); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
file1 = fullfile(out_dir, sprintf('%s_%s.png', stem, stamp));
file2 = fullfile(out_dir, sprintf('%s_latest.png', stem));
try
    exportgraphics(fig, file1);
    exportgraphics(fig, file2);
catch
    saveas(fig, file1);
    saveas(fig, file2);
end
end
