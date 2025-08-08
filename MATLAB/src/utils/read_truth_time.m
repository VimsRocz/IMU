function t = read_truth_time(truth_path, notes)
% READ_TRUTH_TIME Read STATE_* truth file robustly.
%   T = READ_TRUTH_TIME(TRUTH_PATH, NOTES) reads the truth file at
%   TRUTH_PATH ignoring lines that start with '#', splits on whitespace,
%   coerces the first column to numeric, drops NaNs, and returns time
%   starting at zero. If parsing fails a note is appended to NOTES and
%   T is returned empty.
%
%   Inputs:
%       truth_path - path to truth file
%       notes      - cell array of notes (in/out)
%
%   Outputs:
%       t - time vector starting at zero or [] on failure

if nargin < 2
    notes = {};
end

if nargin < 1 || isempty(truth_path) || ~isfile(truth_path)
    t = [];
    return;
end

try
    opts = detectImportOptions(truth_path, 'FileType','text');
    opts = setvaropts(opts, opts.VariableNames, ...
                      'WhitespaceRule','preserve', 'EmptyFieldRule','auto');
    opts.CommentStyle = '#';
    Ttruth = readtable(truth_path, opts);
    Ttruth = Ttruth(~all(ismissing(Ttruth),2), :);
    col = Ttruth{:,1};
    col = col(isfinite(col));
catch
    notes{end+1} = 'TRUTH: failed to parse time column.';
    t = [];
    return;
end

if numel(col) < 2
    notes{end+1} = 'TRUTH: failed to parse time column; insufficient numeric rows.';
    t = [];
    return;
end

t = col - col(1);
end
