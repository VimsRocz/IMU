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
    opts = detectImportOptions(filename, 'FileType','text');
    opts = setvaropts(opts, opts.VariableNames, ...
                      'WhitespaceRule','preserve', 'EmptyFieldRule','auto');
    opts.CommentStyle = '#';
    T = readtable(filename, opts);
    % Drop rows that are completely empty (all NaN)
    T = T(~all(ismissing(T),2), :);
    data = table2array(T);
catch ME
    error('read_state_file:ReadFailed','Failed to read %s: %s', filename, ME.message);
end
end
