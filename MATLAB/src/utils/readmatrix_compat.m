function data = readmatrix_compat(filename, varargin)
%READMATRIX_COMPAT Read matrix from file with MATLAB/Octave compatibility
%   DATA = READMATRIX_COMPAT(FILENAME) reads numeric data from a file.
%   This function provides compatibility between MATLAB's readmatrix and
%   Octave's csvread/dlmread functions.
%
%   DATA = READMATRIX_COMPAT(FILENAME, 'FileType', 'text') reads a text file.
%   
%   This is a simplified compatibility function that handles the most common
%   use cases in this repository.

    if exist('readmatrix', 'builtin') == 5 || exist('readmatrix', 'file') == 2
        % MATLAB - use native readmatrix function
        data = readmatrix(filename, varargin{:});
    else
        % Octave - use alternative functions
        try
            % Try csvread first for CSV files
            if contains(lower(filename), '.csv')
                data = csvread(filename);
            else
                % Try dlmread for other text files
                data = dlmread(filename);
            end
        catch
            try
                % Fallback to load for .dat files or other formats
                data = load(filename);
            catch ME
                error('readmatrix_compat:ReadError', 'Could not read file %s: %s', filename, ME.message);
            end
        end
    end
end