function rebuild_custom_fig(infile, outfile)
%REBUILD_CUSTOM_FIG Recreate a true MATLAB .fig from a custom MAT-based fig.
%  These .fig files were saved from Python by serialising plotted line data
%  into a MAT-file with variables named like:
%    ax<i>_title, ax<i>_xlabel, ax<i>_ylabel
%    ax<i>_line<j>_x, ax<i>_line<j>_y, ax<i>_line<j>_label
%  This function loads that MAT-file, replots each axes, and saves a real
%  MATLAB figure that opens with openfig/double-click in MATLAB.
%
%  rebuild_custom_fig(infile, outfile)
%
%  Example:
%    rebuild_custom_fig('results/foo.fig', 'results/foo_matlab.fig')

data = load(infile, '-mat');
vars = fieldnames(data);

% Discover axes indices present in the MAT-file
ax_idx = [];
for k = 1:numel(vars)
    tok = regexp(vars{k}, '^ax(\d+)_', 'tokens', 'once');
    if ~isempty(tok)
        ax_idx(end+1) = str2double(tok{1}); %#ok<AGROW>
    end
end
ax_idx = unique(ax_idx);
if isempty(ax_idx)
    error('No axes data found in %s', infile);
end

% Choose a reasonable subplot layout
n = numel(ax_idx);
switch n
    case 1, rows = 1; cols = 1;
    case 2, rows = 2; cols = 1;
    case 3, rows = 3; cols = 1;
    case 4, rows = 2; cols = 2;
    case 6, rows = 2; cols = 3;
    case 9, rows = 3; cols = 3;
    otherwise
        rows = n; cols = 1;
end

fig = figure('Color', 'w');
for i = 1:n
    axn = ax_idx(i);
    subplot(rows, cols, i);
    hold on; grid on;
    % Plot all lines for this axes
    j = 1; labels = {};
    while true
        xname = sprintf('ax%d_line%d_x', axn, j);
        yname = sprintf('ax%d_line%d_y', axn, j);
        if ~isfield(data, xname) || ~isfield(data, yname)
            break;
        end
        x = data.(xname); y = data.(yname);
        plot(x, y, 'LineWidth', 1.0);
        lname = sprintf('ax%d_line%d_label', axn, j);
        if isfield(data, lname)
            lab = data.(lname);
            labels{end+1} = local_aschar(lab); %#ok<AGROW>
        end
        j = j + 1;
    end

    % Titles and labels
    tname = sprintf('ax%d_title', axn);
    xlabn = sprintf('ax%d_xlabel', axn);
    ylabn = sprintf('ax%d_ylabel', axn);
    if isfield(data, tname), title(local_aschar(data.(tname))); end
    if isfield(data, xlabn), xlabel(local_aschar(data.(xlabn))); end
    if isfield(data, ylabn), ylabel(local_aschar(data.(ylabn))); end
    if ~isempty(labels)
        legend(labels, 'Interpreter', 'none');
    end
    hold off;
end

% Save a true MATLAB .fig that opens natively
[outdir,~,~] = fileparts(outfile);
if ~isempty(outdir) && ~exist(outdir, 'dir')
    mkdir(outdir);
end
savefig(fig, outfile);

end

function s = local_aschar(val)
%LOCAL_ASCHAR Convert loaded MAT string/cell/char to a char row vector.
    if isstring(val)
        s = char(val);
    elseif iscell(val)
        if isempty(val)
            s = '';
        else
            s = local_aschar(val{1});
        end
    elseif ischar(val)
        s = val;
    else
        try
            s = char(string(val));
        catch
            s = '';
        end
    end
end
