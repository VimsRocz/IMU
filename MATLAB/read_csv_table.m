function T = read_csv_table(filename)
%READ_CSV_TABLE Read CSV file into a structure (Octave-compatible replacement for readtable)
%   T = READ_CSV_TABLE(FILENAME) reads a CSV file and returns a structure
%   with field names from the header row and data columns as arrays.
%
%   This function provides basic readtable functionality for Octave compatibility.

    % Open file and read first line for headers
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    
    % Read header line
    header_line = fgetl(fid);
    if header_line == -1
        fclose(fid);
        error('Empty file or cannot read header: %s', filename);
    end
    
    % Parse headers (split by comma)
    headers = strsplit(header_line, ',');
    num_cols = length(headers);
    
    % Clean header names (remove spaces, make valid field names)
    for i = 1:num_cols
        headers{i} = strtrim(headers{i});
        % Replace invalid characters with underscore
        headers{i} = regexprep(headers{i}, '[^a-zA-Z0-9_]', '_');
        % Ensure it starts with a letter
        if ~isletter(headers{i}(1))
            headers{i} = ['col_' headers{i}];
        end
    end
    
    % Read the rest of the data
    data_text = fileread(filename);
    fclose(fid);
    
    % Split into lines and skip header
    lines = strsplit(data_text, '\n');
    data_lines = lines(2:end);
    
    % Remove empty lines
    data_lines = data_lines(~cellfun(@isempty, data_lines));
    
    num_rows = length(data_lines);
    if num_rows == 0
        error('No data rows found in file: %s', filename);
    end
    
    % Initialize data storage
    data = cell(num_rows, num_cols);
    
    % Parse each data line
    for i = 1:num_rows
        values = strsplit(data_lines{i}, ',');
        if length(values) ~= num_cols
            warning('Row %d has %d columns, expected %d', i, length(values), num_cols);
            continue;
        end
        data(i, :) = values;
    end
    
    % Create output structure
    T = struct();
    
    % Convert data to appropriate types and assign to structure
    for col = 1:num_cols
        col_data = data(:, col);
        
        % Try to convert to numeric
        numeric_data = str2double(col_data);
        
        % If all values are numeric, use numeric array
        if all(~isnan(numeric_data))
            T.(headers{col}) = numeric_data;
        else
            % Keep as cell array of strings
            T.(headers{col}) = col_data;
        end
    end
end