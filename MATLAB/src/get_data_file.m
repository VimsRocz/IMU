function path = get_data_file(filename)
    %GET_DATA_FILE  Return the full path to a data file.
    %   PATH = GET_DATA_FILE(FILENAME) searches common locations within the
    %   repository so scripts remain independent of the current working
    %   directory.  The following folders are checked in order:
    %
    %   ``MATLAB/data``
    %   ``data`` (repository root)
    %   repository root
    %   ``tests/data`` (bundled unit-test logs)
    %
    %   An error is raised if the file cannot be found.

    script_dir = fileparts(mfilename('fullpath'));
    root_dir   = fileparts(script_dir);

    candidates = {
        fullfile(script_dir, 'data', filename), ...
        fullfile(root_dir, 'data', filename),  ...
        fullfile(root_dir, filename),          ...
        fullfile(root_dir, 'tests', 'data', filename)
    };

    for i = 1:numel(candidates)
        if exist(candidates{i}, 'file')
            path = candidates{i};
            return;
        end
    end

    error('Data file not found: %s', filename);
end
