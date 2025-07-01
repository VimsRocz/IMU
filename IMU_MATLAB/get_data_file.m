function path = get_data_file(filename)
    %GET_DATA_FILE Returns the full path to a data file.
    %   Searches in MATLAB/data and repository root.
    script_dir = fileparts(mfilename('fullpath'));
    p1 = fullfile(script_dir, 'data', filename);
    if exist(p1, 'file')
        path = p1;
        return;
    end
    root_dir = fileparts(script_dir);
    p2 = fullfile(root_dir, filename);
    if exist(p2, 'file')
        path = p2;
        return;
    end
    error('Data file not found: %s', filename);
end
