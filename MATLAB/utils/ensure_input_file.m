function abs_path = ensure_input_file(kind, fname, paths)
%ENSURE_INPUT_FILE Locate or copy an input data file to the project root.
%   abs_path = ENSURE_INPUT_FILE(KIND, FNAME, PATHS) returns the absolute
%   path. KIND is 'IMU', 'GNSS' or 'TRUTH'. If the file is missing under
%   PATHS.root, common locations are searched and the file is copied into
%   the root for stability.
%
% Usage:
%   imu_path = ensure_input_file('IMU', 'IMU_X002.dat', paths);

% Fallbacks if 'paths' is missing fields (paranoid safe)
if ~isfield(paths,'root') || isempty(paths.root)
    paths.root = fileparts(fileparts(mfilename('fullpath'))); % .../MATLAB -> root
end
if ~isfield(paths,'matlab') || isempty(paths.matlab)
    paths.matlab = fullfile(paths.root,'MATLAB');
end

target = fullfile(paths.root, fname);
if isfile(target), abs_path = target; return; end

search = {
    fullfile(paths.root, fname)
    fullfile(paths.matlab, fname)
    fullfile(paths.root, 'MATLAB', 'src', fname)
    fullfile(paths.root, 'src', fname)
    fullfile(paths.root, 'IMU', fname)  % common local data folder
    fullfile(paths.root, 'Data', fname) % alternative data folder
    fullfile(paths.root, '..', fname)
    fullfile(paths.root, 'MATLAB', fname)
};

src_found = '';
for i = 1:numel(search)
    if isfile(search{i}), src_found = search{i}; break; end
end

if ~isempty(src_found)
    try
        copyfile(src_found, target);
        fprintf('Copied %s: %s -> %s\n', kind, src_found, target);
        abs_path = target;
    catch ME
        warning('Could not copy %s (%s). Using source in-place.', kind, ME.message);
        abs_path = src_found;
    end
    return;
end

error('%s file not found. Looked for %s and in:\n  - %s', kind, target, strjoin(search, sprintf('\n  - ')));
end
