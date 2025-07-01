function path = get_data_file(filename)
    %GET_DATA_FILE Returns the full path to a data file.
    %   Searches in Data/, MATLAB/data and repository root. Wildcard patterns
    %   return a cell array of all matching files.

    script_dir = fileparts(mfilename('fullpath'));
    search_dirs = {fullfile(script_dir, '..', 'Data'), ...
                   fullfile(script_dir, 'data'), ...
                   fileparts(script_dir)};

    has_wildcard = ~isempty(regexp(filename, '[*?]', 'once'));
    matches = {};
    for k = 1:numel(search_dirs)
        candidate = fullfile(search_dirs{k}, filename);
        if has_wildcard
            d = dir(candidate);
            matches = [matches, arrayfun(@(f) fullfile(f.folder, f.name), d, 'UniformOutput', false)]; %#ok<AGROW>
        else
            if exist(candidate, 'file')
                path = candidate;
                return;
            end
        end
    end

    if has_wildcard && ~isempty(matches)
        path = matches;
        return;
    end

    error('Data file not found: %s', filename);
end
