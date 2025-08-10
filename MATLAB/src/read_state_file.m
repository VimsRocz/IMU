function data = read_state_file(filename)
%READ_STATE_FILE Load ground truth STATE_X file.
%   DATA = READ_STATE_FILE(FILENAME) reads the numeric columns from the
%   specified state file. Lines beginning with '#' are ignored so the
%   function is robust to header comments. Returns a numeric matrix with
%   one row per sample.
%
%   Mirrors the behaviour of ``np.loadtxt`` used in the Python
%   implementation.

if nargin < 1
    error('read_state_file:MissingFile','Filename required');
end

if ~isfile(filename)
    error('read_state_file:FileNotFound','File not found: %s', filename);
end

% Robust import that skips comment lines and blanks
try
    opts = detectImportOptions(filename, 'FileType','text', ...
                               'Delimiter',' ', ...
                               'ConsecutiveDelimitersRule','join');
    % Preserve original column headers to avoid noisy warnings
    if isprop(opts, 'VariableNamingRule')
        opts.VariableNamingRule = 'preserve';
    end
    opts.CommentStyle = '#';
    % Avoid per-variable varopts to support older MATLAB versions
    T = readtable(filename, opts);
    % Drop rows that are completely empty (all NaN)
    T = T(~all(ismissing(T),2), :);
    data = table2array(T);
catch ME
    % Fallback: use readmatrix with comment handling via textscan
    try
        fid = fopen(filename,'r');
        C = textscan(fid, '%f', 'CommentStyle','#', 'MultipleDelimsAsOne',true);
        fclose(fid);
        v = C{1};
        % Infer column count from first non-comment line
        fid = fopen(filename,'r');
        ncols = [];
        while ~feof(fid)
            ln = fgetl(fid);
            if ischar(ln) && ~startsWith(strtrim(ln),'#') && ~isempty(strtrim(ln))
                ncols = numel(str2num(ln)); %#ok<ST2NM>
                break;
            end
        end
        fclose(fid);
        if isempty(ncols) || mod(numel(v), ncols) ~= 0
            error('read_state_file:ParseFailed','Could not infer consistent columns.');
        end
        data = reshape(v, [ncols, numel(v)/ncols]).';
    catch
        error('read_state_file:ReadFailed','Failed to read %s: %s', filename, ME.message);
    end
end
end
