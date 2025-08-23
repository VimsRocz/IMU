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
    vn = T.Properties.VariableNames;
    for i = 1:numel(vn)
        col = T.(vn{i});
        if iscell(col) || isstring(col)
            T.(vn{i}) = str2double(string(col));
        end
    end
    % Build time vector starting at zero
    time_col = [];
    if any(strcmpi(vn,'Posix_Time'))
        t_raw = T{:,strcmpi(vn,'Posix_Time')};
    elseif any(strcmpi(vn,'time'))
        t_raw = T{:,strcmpi(vn,'time')};
    else
        t_raw = T{:,1};
    end
    t_raw = t_raw(:);
    % Normalize units: some STATE files store 0.1s ticks in column 2
    dt_med = median(diff(t_raw));
    if isfinite(dt_med) && dt_med > 0.5 && dt_med < 1.5
        t_truth = t_raw / 10;   % convert 0.1s ticks to seconds (~10 Hz)
    else
        t_truth = t_raw;        % already seconds
    end
    t = t_truth - t_truth(1);
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
