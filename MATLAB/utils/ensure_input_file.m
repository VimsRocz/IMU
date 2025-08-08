function abs_path = ensure_input_file(kind, fname, paths)
%ENSURE_INPUT_FILE Locate or copy an input data file to the project root.
%   abs_path = ENSURE_INPUT_FILE(KIND, FNAME, PATHS) returns the absolute
%   path to FNAME. If the file does not exist under PATHS.root, common
%   locations are searched and the file is copied to the root for future
%   runs.
%
%   KIND is only used for printed diagnostics (e.g. 'IMU', 'GNSS').
%
%   Usage:
%       imu_path = ensure_input_file('IMU', 'IMU_X002.dat', paths);

if nargin < 3
    error('ensure_input_file:MissingInput', 'paths struct required');
end

target = fullfile(paths.root, fname);
if isfile(target)
    abs_path = target; return;
end

search = {
    fullfile(paths.root, fname)
    fullfile(paths.matlab, fname)
    fullfile(paths.root, 'MATLAB', 'src', fname)
    fullfile(paths.root, 'src', fname)
    fullfile(paths.root, '..', fname)
    fullfile(paths.root, 'MATLAB', fname)
};

src_found = '';
for i = 1:numel(search)
    if isfile(search{i})
        src_found = search{i};
        break;
    end
end

if ~isempty(src_found)
    try
        copyfile(src_found, target);
        fprintf('Copied %s from %s -> %s\n', kind, src_found, target);
    catch ME
        warning('Could not copy %s from %s -> %s (%s). Using source in place.', kind, src_found, target, ME.message);
        abs_path = src_found; return;
    end
    abs_path = target;
    return;
end

error('%s file not found. Looked for %s and in: %s', kind, target, strjoin(search, ', '));
end

