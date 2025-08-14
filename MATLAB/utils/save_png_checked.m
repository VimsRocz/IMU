function save_png_checked(fig, filename)
%SAVE_PNG_CHECKED Save figure as PNG with fixed size and verify output.
%   SAVE_PNG_CHECKED(FIG, FILENAME) writes FIG to FILENAME as a PNG using
%   1800x1200 pixels at 200 DPI in landscape orientation. The function
%   asserts that the file is created and larger than 5 KB.
%
%   Example:
%       save_png_checked(gcf, 'MATLAB/results/demo.png');
%
%   The function creates parent directories if necessary.

    if nargin < 1 || isempty(fig)
        fig = gcf;
    end
    if nargin < 2 || isempty(filename)
        error('save_png_checked:MissingFile', 'Filename is required');
    end
    [out_dir, ~, ext] = fileparts(filename);
    if isempty(ext)
        filename = [filename '.png'];
        ext = '.png';
    end
    if ~strcmpi(ext, '.png')
        error('save_png_checked:Extension', 'Filename must end with .png');
    end
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    % Configure figure dimensions
    set(fig, 'Units', 'pixels');
    set(fig, 'Position', [100 100 1800 1200]);
    set(fig, 'PaperOrientation', 'landscape');
    set(fig, 'PaperUnits', 'inches', 'PaperPosition', [0 0 9 6]);

    % Save using print for consistent dpi
    print(fig, filename, '-dpng', '-r200');

    info = dir(filename);
    assert(~isempty(info), 'save_png_checked:FileMissing', ...
        'Failed to create %s', filename);
    assert(info.bytes > 5*1024, 'save_png_checked:FileTooSmall', ...
        'File %s is only %d bytes', filename, info.bytes);
    fprintf('Saved %s (%d bytes)\n', filename, info.bytes);
end
