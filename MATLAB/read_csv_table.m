function tbl = read_csv_table(filename)
%READ_CSV_TABLE  Read CSV file as table-like structure compatible with Octave
%   This function replaces readtable for Octave compatibility.
%   Returns a struct with fields matching the CSV column headers.

    % Read the first line to get column headers
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    
    header_line = fgetl(fid);
    fclose(fid);
    
    % Parse headers
    headers = strsplit(header_line, ',');
    
    % Clean headers (remove whitespace and make valid field names)
    for i = 1:length(headers)
        headers{i} = strtrim(headers{i});
        % Replace invalid characters with underscores
        headers{i} = regexprep(headers{i}, '[^a-zA-Z0-9_]', '_');
    end
    
    % Read data matrix (skip header)
    try
        data = dlmread(filename, ',', 1, 0);
    catch
        % If dlmread fails, try csvread
        data = csvread(filename, 1, 0);
    end
    
    % Create table-like structure
    tbl = struct();
    for i = 1:length(headers)
        if i <= size(data, 2)
            tbl.(headers{i}) = data(:, i);
        end
    end
    
    % Add Properties field to mimic MATLAB table behavior
    tbl.Properties = struct();
    tbl.Properties.VariableNames = headers;
end