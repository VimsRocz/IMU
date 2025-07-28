function data = read_matrix(filename, varargin)
%READ_MATRIX Read matrix data from file (Octave-compatible replacement for readmatrix)
%   DATA = READ_MATRIX(FILENAME) reads numeric data from a text file.
%   
%   This function provides basic readmatrix functionality for Octave compatibility.
%   Supports common options like 'CommentStyle'.

    p = inputParser();
    addParameter(p, 'CommentStyle', '', @ischar);
    parse(p, varargin{:});
    
    comment_style = p.Results.CommentStyle;
    
    if ~isempty(comment_style)
        % Read file line by line, filtering out comments
        fid = fopen(filename, 'r');
        if fid == -1
            error('Cannot open file: %s', filename);
        end
        
        lines = {};
        while ~feof(fid)
            line = fgetl(fid);
            if ischar(line) && ~isempty(line)
                % Skip comment lines
                if ~isempty(comment_style) && strncmp(strtrim(line), comment_style, length(comment_style))
                    continue;
                end
                lines{end+1} = line;
            end
        end
        fclose(fid);
        
        % Write filtered content to temporary file
        temp_file = tempname();
        fid = fopen(temp_file, 'w');
        for i = 1:length(lines)
            fprintf(fid, '%s\n', lines{i});
        end
        fclose(fid);
        
        % Read the filtered data
        try
            data = dlmread(temp_file);
        catch
            % Fallback to csvread
            data = csvread(temp_file);
        end
        
        % Clean up temporary file
        delete(temp_file);
    else
        % Direct read for files without comments
        try
            data = dlmread(filename);
        catch
            % Fallback to csvread
            data = csvread(filename);
        end
    end
end