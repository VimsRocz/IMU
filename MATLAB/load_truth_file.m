function T = load_truth_file(truth_path)
%LOAD_TRUTH_FILE Robustly load truth file and return table with time_s.
%   T = LOAD_TRUTH_FILE(PATH) reads STATE_X001-style files with space/tab
%   delimiters, ignores '#' comments, joins repeated delimiters and coerces
%   string columns to numeric. Adds T.time_s starting at zero.

    if nargin < 1 || isempty(truth_path)
        error('load_truth_file:MissingPath','truth_path is required');
    end
    if ~isfile(truth_path)
        error('load_truth_file:NotFound','Truth file not found: %s', truth_path);
    end

    opts = delimitedTextImportOptions('Delimiter',{'\t',' '}, ...
                                      'ConsecutiveDelimitersRule','join', ...
                                      'LeadingDelimitersRule','ignore');
    opts.CommentStyle = '#';
    opts.ExtraColumnsRule = 'ignore';
    opts.EmptyLineRule = 'read';
    try
        T = readtable(truth_path, opts);
    catch
        % Fallback using detectImportOptions for older MATLAB versions
        o2 = detectImportOptions(truth_path,'Delimiter',' ', ...
                                 'ConsecutiveDelimitersRule','join');
        o2.CommentStyle = '#';
        T = readtable(truth_path, o2);
    end

    % Coerce non-numeric columns to numeric when possible
    for i = 1:width(T)
        if ~isnumeric(T{:,i})
            T{:,i} = str2double(string(T{:,i}));
        end
    end
    % Build time vector starting at zero
    time_col = [];
    vn = T.Properties.VariableNames;
    if any(strcmpi(vn,'Posix_Time'))
        t = T{:,strcmpi(vn,'Posix_Time')};
    elseif any(strcmpi(vn,'time'))
        t = T{:,strcmpi(vn,'time')};
    else
        t = T{:,1};
    end
    t = t(:);
    t = t - t(1);
    T.time_s = t;

    % Print a timeline summary line mirroring Python
    d = diff(T.time_s);
    if isempty(d) || any(~isfinite(d))
        hz = NaN; dt_med = NaN; dur = NaN; t0 = NaN; t1 = NaN; mono = false;
    else
        dt_med = median(d);
        hz = 1/dt_med;
        dur = T.time_s(end) - T.time_s(1);
        t0 = T.time_s(1); t1 = T.time_s(end);
        mono = all(d > 0);
    end
    fprintf('TRUTH | n=%d hz=%0.6f dt_med=%0.6f dur=%0.3f t0=%0.6f t1=%0.6f monotonic=%s\n', ...
            height(T), hz, dt_med, dur, T.time_s(1), T.time_s(end), string(mono));
end

