function save_validated_png(fig, filename)
%SAVE_VALIDATED_PNG Save figure as PNG with size check.
%   SAVE_VALIDATED_PNG(FIG, FILENAME) writes FIG to FILENAME using
%   1800x1200 pixels at 200 DPI in landscape orientation and verifies the
%   file exists and is larger than 5 KB.
%
%   Example:
%       save_validated_png(gcf, 'MATLAB/results/demo.png');

    if nargin < 1 || isempty(fig)
        fig = gcf;
    end
    if nargin < 2 || isempty(filename)
        error('save_validated_png:MissingFile', 'Filename is required');
    end
    [out_dir, ~, ext] = fileparts(filename);
    if isempty(ext)
        filename = [filename '.png'];
        ext = '.png';
    end
    if ~strcmpi(ext, '.png')
        error('save_validated_png:Extension', 'Filename must end with .png');
    end
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    set(fig, 'Units', 'pixels', 'Position', [100 100 1800 1200]);
    set(fig, 'PaperOrientation', 'landscape');
    set(fig, 'PaperUnits', 'inches', 'PaperPosition', [0 0 9 6]);

    print(fig, filename, '-dpng', '-r200');

    info = dir(filename);
    assert(~isempty(info), 'save_validated_png:FileMissing', ...
        'Failed to create %s', filename);
    assert(info.bytes > 5*1024, 'save_validated_png:FileTooSmall', ...
        'File %s is only %d bytes', filename, info.bytes);
    fprintf('Saved %s (%d bytes)\n', filename, info.bytes);
end
