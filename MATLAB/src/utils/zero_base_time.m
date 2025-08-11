function t0 = zero_base_time(t)
%ZERO_BASE_TIME Accept datetime or numeric, return double seconds starting at 0.
%   T0 = ZERO_BASE_TIME(T) returns a column vector in seconds where the
%   first valid entry is zero and subsequent entries are relative to this
%   time. ``T`` may be datetime or numeric. Leading non-finite values are
%   discarded. An error is raised if no finite times remain.
%
%   This mirrors the Python ``zero_base_time`` helper.

    if isempty(t)
        error('zero_base_time:empty','Empty time vector');
    end
    if isdatetime(t)
        t = posixtime(t);
    end
    t = double(t(:));
    mask = isfinite(t);
    if ~any(mask)
        error('zero_base_time:nonfinite','No finite times');
    end
    % drop leading non-finite
    first = find(mask,1);
    t = t(first:end);
    t0 = t - t(1);
end
